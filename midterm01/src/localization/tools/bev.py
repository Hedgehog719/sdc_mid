"""Bird's-eye-view rasters for map-to-image localization.

Vehicle BEV frame: x forward, y left, origin below the camera on the road plane; pixel (r, c) with
r = (X_MAX - x) / RES, c = (Y_MAX - y) / RES.
  image_bev(): inverse perspective mapping of the fisheye images onto the road plane (calibrated K, D,
               camera extrinsics), with YOLO road-user masks and the hood mask removed.
  map_bev():   ground-intensity raster of the LiDAR map around a pose.
  lane_response(): white top-hat (thin bright markings) normalised by local contrast.
"""
import os

import cv2
import numpy as np

from camera_model import Cameras, R_OPT_FLU

RES = 0.1
X_MIN, X_MAX = -5.0, 30.0   # BEV from 5 m behind to 30 m ahead of the front camera: the side cameras see the
                            # road beside the car (kerbs, adjacent lane lines) closest and sharpest
NEAR = 3.0                  # start of the area ahead of the car used by road-surface heuristics
Y_MAX = 15.0
H = int((X_MAX - X_MIN) / RES)
W = int(2 * Y_MAX / RES)


def local_normalise(g, ok, k=31):
    """Zero-mean / unit-std within a k x k window, computed only over this camera's valid pixels, so the three
    cameras' different exposure and white balance do not leave seams (false edges) in the blended BEV."""
    w = ok.astype(np.float32)
    gw = np.where(ok, g, 0).astype(np.float32)
    den = cv2.blur(w, (k, k)) + 1e-6
    mean = cv2.blur(gw, (k, k)) / den
    var = cv2.blur(gw * gw, (k, k)) / den - mean ** 2
    out = (g - mean) / np.sqrt(np.maximum(var, 1.0))
    return np.where(ok, out, 0).astype(np.float32)


def bev_grid():
    r, c = np.mgrid[0:H, 0:W]
    x = X_MAX - (r + 0.5) * RES
    y = Y_MAX - (c + 0.5) * RES
    return x, y


CAM_SYNC = os.environ.get('SDC_CAM_SYNC', '0') == '1'   # compensate the side cameras' capture-time offsets


def camera_time_offsets(track_dir, cams=('f', 'fl', 'fr')):
    """Per camera, per frame capture-time offset (s) w.r.t. the front camera (frame) timestamp, from the image file
    names (<idx>_ts_<ns>.jpg). Track2: fl -9.8 ms, fr +13.3 ms; Track1: fl -1.9 ms, fr +12.3 ms."""
    track = os.path.basename(os.path.normpath(track_dir))
    def ts(c):
        d = os.path.join(track_dir, f'{track}_images', f'{track}_{c}_images')
        return np.array([int(f.split('_ts_')[1].split('.')[0]) for f in sorted(os.listdir(d))], dtype=np.int64)
    t0 = ts('f')
    out = {}
    for c in cams:
        t = ts(c) if c != 'f' else t0
        n = min(len(t), len(t0))
        dt = np.zeros(len(t0))
        dt[:n] = (t[:n] - t0[:n]) * 1e-9
        out[c] = dt
    return out


def shift_rows(img, d_m, interp=cv2.INTER_LINEAR):
    """BEV content of a camera that captured d_m metres further along the path: R_frame(x) = R_cam(x - d)."""
    if abs(d_m) < 1e-4:
        return img
    M = np.float32([[1, 0, 0], [0, 1, -d_m / RES]])
    return cv2.warpAffine(img, M, (img.shape[1], img.shape[0]), flags=interp, borderMode=cv2.BORDER_CONSTANT, borderValue=0)


class ImageBEV:
    def __init__(self, track_dir, cam_height=2.1, pitch_deg=0.0, roll_deg=0.0, yaw_deg=0.0, cams=('f', 'fl', 'fr')):
        track = os.path.basename(os.path.normpath(track_dir))
        self.track_dir, self.track = track_dir, track
        self.C = Cameras(os.path.join(track_dir, f'{track}_images', 'camera_info'))
        self.cams = cams
        self.names = {c: sorted(os.listdir(os.path.join(track_dir, f'{track}_images', f'{track}_{c}_images'))) for c in cams}
        hood = cv2.imread(os.path.join(track_dir, f'{track}_images', 'camera_info', 'gige_100_f_hdr_mask.png'), cv2.IMREAD_UNCHANGED)
        self.hood = None
        if hood is not None:
            a = hood if hood.ndim == 2 else hood[..., 0]  # white (RGB) = ego hood; alpha is constant
            self.hood = a > 127
        # hand-drawn ego-vehicle masks (pole_review_server /hood -> tools/ego_mask.py), same car for both tracks:
        # front replaces the supplied hood mask, side cameras get their visible body corners masked
        self.ego = {}
        if USE_EGO_MASK:
            root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
            for c in cams:
                f = os.path.join(root, 'calib', f'ego_mask_{c}.png')
                if not os.path.exists(f):
                    f = os.path.join(root, 'work', f'ego_mask_{c}.png')
                if os.path.exists(f):
                    self.ego[c] = cv2.imread(f, cv2.IMREAD_GRAYSCALE) > 0
            if 'f' in self.ego:
                self.hood = self.ego['f']
        self.masks = {}
        mdir = os.path.join(track_dir, 'masks')
        for c in cams:
            f = os.path.join(mdir, f'{c}.npz')
            if os.path.exists(f):
                self.masks[c] = np.load(f)['masks']
        self.calib = (cam_height, pitch_deg, roll_deg, yaw_deg)
        self.dyn = None  # optional (N, 2) per-frame dynamic pitch/roll offsets in degrees
        self.photonorm = False  # per-camera local mean/std normalisation before blending (exposure / white balance)
        self.set_calib(cam_height, pitch_deg, roll_deg, yaw_deg)

    def ego_bad(self, c, shape):
        """uint8 mask of the own vehicle in camera c (front: hood; sides: body corners), zeros if none."""
        m = self.ego.get(c)
        if m is None and c == 'f':
            m = self.hood
        if m is None or m.shape != tuple(shape[:2]):
            return np.zeros(shape[:2], np.uint8)
        return m.astype(np.uint8)

    def set_calib(self, cam_height, pitch_deg, roll_deg, yaw_deg):
        from scipy.spatial.transform import Rotation
        x, y = bev_grid()
        pts = np.stack([x.ravel(), y.ravel(), np.full(x.size, -cam_height)], 1)
        # small mounting rotation of the camera rig w.r.t. the road-aligned frame
        Rm = Rotation.from_euler('ZYX', [yaw_deg, pitch_deg, roll_deg], degrees=True).as_matrix()
        pts = pts @ Rm  # = Rm^T p
        self.maps = {}
        for c in self.cams:
            uv, valid = self.C.project(c, pts)
            mx = uv[:, 0].reshape(H, W).astype(np.float32)
            my = uv[:, 1].reshape(H, W).astype(np.float32)
            v = valid.reshape(H, W)
            mx[~v] = -1
            my[~v] = -1
            self.maps[c] = (mx, my, v)

    def render_resp_color(self, i, fn):
        """Like render_resp, but the marking response is the max over luminance, yellowness ((R+G)/2-B) and
        redness (R-(G+B)/2): yellow lane / island lines and red kerb lines have little luminance contrast on asphalt
        but are clear in colour, and painted lines are visible in the LiDAR reflectivity."""
        if self.dyn is not None:
            h, p, r, y = self.calib
            self.set_calib(h, p + self.dyn[i, 0], r + self.dyn[i, 1], y)
        acc = np.zeros((H, W), np.float32)
        cnt = np.zeros((H, W), np.float32)
        for c in self.cams:
            img = cv2.imread(os.path.join(self.track_dir, f'{self.track}_images', f'{self.track}_{c}_images', self.names[c][i]))
            if img is None:
                continue
            bad = np.zeros(img.shape[:2], np.uint8)
            if c in self.masks:
                m = np.unpackbits(self.masks[c][i], axis=1)[:, :img.shape[1] // 2]
                bad |= cv2.resize(m, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
            bad |= self.ego_bad(c, bad.shape)
            mx, my, v = self.maps[c]
            col = cv2.remap(img, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0).astype(np.float32)
            b = cv2.remap(bad, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=1)
            ok = v & (b == 0)
            if ok.sum() < 100:
                continue
            B_, G_, R_ = col[..., 0], col[..., 1], col[..., 2]
            lum = 0.114 * B_ + 0.587 * G_ + 0.299 * R_
            yel = np.maximum((R_ + G_) / 2 - B_, 0)
            red = np.maximum(R_ - (G_ + B_) / 2, 0)
            r_ = np.maximum.reduce([fn(lum, ok), fn(yel, ok), fn(red, ok)])
            okr = cv2.erode(ok.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
            acc[okr] += r_[okr]
            cnt[okr] += 1
        valid = cnt > 0
        acc[valid] /= cnt[valid]
        return acc, valid

    tshift = None   # {cam: per-frame along-path shift (m) = speed * capture-time offset}, see set_motion

    def set_motion(self, speed):
        """speed: per-frame vehicle speed (m/s) -> per-camera BEV shift compensating the capture-time offsets."""
        dts = camera_time_offsets(self.track_dir, self.cams)
        self.tshift = {c: np.asarray(speed, float)[:len(dts[c])] * dts[c][:len(speed)] for c in self.cams}

    def _sync(self, c, i, arr, interp=cv2.INTER_LINEAR):
        if self.tshift is None or c not in self.tshift or i >= len(self.tshift[c]):
            return arr
        return shift_rows(arr, float(self.tshift[c][i]), interp)

    def render_resp(self, i, fn):
        """Per-camera feature response fn(gray_c, valid_c) blended afterwards (average where valid): exposure and
        white-balance differences between cameras cannot create seam edges, since each response is computed
        inside one camera's image only."""
        if self.dyn is not None:
            h, p, r, y = self.calib
            self.set_calib(h, p + self.dyn[i, 0], r + self.dyn[i, 1], y)
        acc = np.zeros((H, W), np.float32)
        cnt = np.zeros((H, W), np.float32)
        for c in self.cams:
            img = cv2.imread(os.path.join(self.track_dir, f'{self.track}_images', f'{self.track}_{c}_images', self.names[c][i]),
                             cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            bad = np.zeros(img.shape, np.uint8)
            if c in self.masks:
                m = np.unpackbits(self.masks[c][i], axis=1)[:, :img.shape[1] // 2]
                bad |= cv2.resize(m, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
            bad |= self.ego_bad(c, img.shape)
            mx, my, v = self.maps[c]
            g = cv2.remap(img, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0).astype(np.float32)
            b = cv2.remap(bad, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=1)
            ok = v & (b == 0)
            if ok.sum() < 100:
                continue
            r_ = fn(g, ok)
            okr = cv2.erode(ok.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0  # this camera's interior only
            if self.tshift is not None:
                r_ = self._sync(c, i, r_.astype(np.float32))
                okr = self._sync(c, i, okr.astype(np.uint8), cv2.INTER_NEAREST) > 0
            acc[okr] += r_[okr]
            cnt[okr] += 1
        valid = cnt > 0
        acc[valid] /= cnt[valid]
        return acc, valid

    def render(self, i):
        """Return (gray BEV float32, valid mask)."""
        if self.dyn is not None:  # braking / bumps: re-project with this frame's body pitch and roll
            h, p, r, y = self.calib
            self.set_calib(h, p + self.dyn[i, 0], r + self.dyn[i, 1], y)
        acc = np.zeros((H, W), np.float32)
        cnt = np.zeros((H, W), np.float32)
        for c in self.cams:
            img = cv2.imread(os.path.join(self.track_dir, f'{self.track}_images', f'{self.track}_{c}_images', self.names[c][i]),
                             cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            bad = np.zeros(img.shape, np.uint8)
            if c in self.masks:
                m = np.unpackbits(self.masks[c][i], axis=1)[:, :img.shape[1] // 2]
                bad |= cv2.resize(m, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
            bad |= self.ego_bad(c, img.shape)
            mx, my, v = self.maps[c]
            g = cv2.remap(img, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0).astype(np.float32)
            b = cv2.remap(bad, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=1)
            ok = v & (b == 0)
            if self.photonorm:
                g = local_normalise(g, ok)
            acc[ok] += g[ok]
            cnt[ok] += 1
        valid = cnt > 0
        acc[valid] /= cnt[valid]
        return acc, valid


def local_ground(hmin, valid, open_m=1.5, smooth_m=1.0):
    """Smooth local ground surface from the lowest point per BEV cell: holes filled, objects removed by a grey
    opening (open_m), then Gaussian smoothing (smooth_m). Follows grades, crests, dips and crossfall that a single
    plane cannot (e.g. Track1 start: 0.7 m height variation within 30 m)."""
    big = np.where(valid, hmin, 1e4).astype(np.float32)
    h = big.copy()
    for k in (5, 11, 21):  # fill scan gaps / holes with nearby minima
        f = cv2.erode(h, np.ones((k, k), np.uint8))
        h = np.where(h > 1e3, f, h)
    h = np.where(h > 1e3, np.median(hmin[valid]) if valid.any() else 0, h).astype(np.float32)
    ko = max(3, int(open_m / RES) | 1)
    g = cv2.dilate(cv2.erode(h, np.ones((ko, ko), np.uint8)), np.ones((ko, ko), np.uint8))
    return cv2.GaussianBlur(g, (0, 0), smooth_m / RES)


GROUND_MODE = 'plane'
USE_EGO_MASK = True


def road_profile(x, y, z, cross, strip=2.5, bin_m=2.0):
    """Ground height along x (fwd) from the lane strip |y| < strip: low percentile per bin_m bin (crossfall
    removed), gaps interpolated, lightly smoothed. Lateral tolerance stays that of the plane, so pavements and
    parked scooters beside the road are still rejected while longitudinal dips / warps are followed."""
    edges = np.arange(X_MIN - 1.0, X_MAX + bin_m + 1.0, bin_m)
    zc = z - cross * y
    m = np.abs(y) < strip
    k = np.digitize(x[m], edges) - 1
    prof = np.full(len(edges) - 1, np.nan)
    for b in range(len(prof)):
        zb = zc[m][k == b]
        if len(zb) >= 20:
            prof[b] = np.percentile(zb, 10)
    cen = 0.5 * (edges[:-1] + edges[1:])
    ok = np.isfinite(prof)
    if ok.sum() < 2:
        return np.full(len(x), np.percentile(zc, 5))
    prof = np.interp(cen, cen[ok], prof[ok])
    pad = np.r_[prof[0], prof, prof[-1]]
    prof = np.median(np.stack([pad[:-2], pad[1:-1], pad[2:]]), 0)  # 3-bin median against cars in the strip
    return np.interp(x, cen, prof)


PIN_Z = 0.0  # GROUND_MODE 'pinned': assumed road height (map frame)
HYBRID_RATIO = 0.6  # GROUND_MODE 'hybrid': use the local ground when plane road < ratio x local road


def map_bev(xyz, intensity, pose_xy, yaw, plane_tol=0.25, rough_max=0.08):
    """Road-surface intensity BEV; GROUND_MODE 'hybrid' keeps the single plane unless it clearly fails.

    The single plane fails where the map's road surface is warped (Track1 start: the map ground dips ~1 m within
    10 m while the real street is flat), leaving most of the road outside plane_tol. Then the local ground is
    used for the map road mask (the image IPM stays flat, which is right for the real road).
    """
    if GROUND_MODE != 'hybrid':
        return _map_bev(xyz, intensity, pose_xy, yaw, plane_tol, rough_max, GROUND_MODE)
    mp = _map_bev(xyz, intensity, pose_xy, yaw, plane_tol, rough_max, 'plane')
    ml = _map_bev(xyz, intensity, pose_xy, yaw, plane_tol, rough_max, 'local')
    return ml if mp[1].mean() < HYBRID_RATIO * ml[1].mean() else mp


def _map_bev(xyz, intensity, pose_xy, yaw, plane_tol, rough_max, mode):
    """Road-surface intensity BEV of map points around pose (vehicle BEV frame).

    Road = cells whose lowest points lie on the local road plane (fitted under the vehicle) within plane_tol
    and are flat (z spread of near-ground points < rough_max); vegetation and kerbside clutter are rejected.
    Returns (intensity raster, road mask).
    """
    c, s = np.cos(yaw), np.sin(yaw)
    d = xyz[:, :2] - pose_xy
    x = c * d[:, 0] + s * d[:, 1]
    y = -s * d[:, 0] + c * d[:, 1]
    # only the BEV extent (+ the road-profile margin) is used below: drop the rest of the 42 m disc first (where the
    # mapping car stood still the disc holds ~18 M points)
    keep = (x > X_MIN - 1.5) & (x < X_MAX + 3.5) & (np.abs(y) < Y_MAX + 0.5)
    x, y, xyz, intensity = x[keep], y[keep], xyz[keep], np.asarray(intensity)[keep]
    z = xyz[:, 2].astype(np.float64)
    # local road plane from points under the vehicle footprint
    near = (np.abs(x) < 6) & (np.abs(y) < 2.5)
    if near.sum() > 50:
        zn = z[near]
        band = zn < np.percentile(zn, 30) + 0.1
        A = np.stack([x[near][band], y[near][band], np.ones(band.sum())], 1)
        coef = np.linalg.lstsq(A, zn[band], rcond=None)[0]
    else:
        coef = np.array([0.0, 0.0, np.percentile(z, 5)])
    hrel = z - (coef[0] * x + coef[1] * y + coef[2])
    if mode == 'profile':  # road height profile along the driving direction (follows warps of the map ground)
        hrel = z - (road_profile(x, y, z, coef[1]) + coef[1] * y)
    win = plane_tol + 0.3
    if mode == 'pinned':  # road assumed level at PIN_Z (map ground there is corrupted); only points near it count
        hrel = z - PIN_Z
        win = plane_tol
    r = ((X_MAX - x) / RES).astype(np.int64)
    col = ((Y_MAX - y) / RES).astype(np.int64)
    if mode in ('local', 'smooth'):
        inb = (r >= 0) & (r < H) & (col >= 0) & (col < W)
        zmin = np.full(H * W, np.inf)
        np.minimum.at(zmin, r[inb] * W + col[inb], z[inb])
        zmin = zmin.reshape(H, W)
        gsurf = local_ground(zmin.astype(np.float32), np.isfinite(zmin))
        hrel = np.full(len(z), 1e3)
        hrel[inb] = z[inb] - gsurf[r[inb], col[inb]]
    ok = (r >= 0) & (r < H) & (col >= 0) & (col < W) & (np.abs(hrel) < win)
    r, col, hr, it = r[ok], col[ok], hrel[ok], intensity[ok]
    hmin = np.full((H, W), np.inf)
    np.minimum.at(hmin, (r, col), hr)
    nearg = hr < hmin[r, col] + 0.1
    acc = np.zeros((H, W), np.float32)
    cnt = np.zeros((H, W), np.float32)
    s1 = np.zeros((H, W), np.float32)
    s2 = np.zeros((H, W), np.float32)
    rr, cc, hh = r[nearg], col[nearg], hr[nearg]
    np.add.at(acc, (rr, cc), np.clip(it[nearg], 0, 255))
    np.add.at(cnt, (rr, cc), 1)
    np.add.at(s1, (rr, cc), hh)
    np.add.at(s2, (rr, cc), hh * hh)
    valid = cnt > 0
    acc[valid] /= cnt[valid]
    rough = np.zeros((H, W), np.float32)
    rough[valid] = np.sqrt(np.maximum(s2[valid] / cnt[valid] - (s1[valid] / cnt[valid]) ** 2, 0))
    # rough/height computed on a 0.5 m neighbourhood to fill sparse rings
    k = np.ones((5, 5), np.float32)
    cnt_n = cv2.filter2D(valid.astype(np.float32), -1, k)
    hmin_f = np.where(valid, hmin, 0).astype(np.float32)
    hmean_n = cv2.filter2D(hmin_f, -1, k) / np.maximum(cnt_n, 1)
    rough_n = cv2.filter2D(rough, -1, k) / np.maximum(cnt_n, 1)
    road = (cnt_n >= 3) & (np.abs(hmean_n) < plane_tol) & (rough_n < rough_max)
    road = cv2.morphologyEx(road.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)) > 0
    if mode == 'smooth':  # road surface = smooth ground connected to the vehicle (local slope criterion)
        rs, kn = map_road_smooth(xyz.astype(np.float64), pose_xy, yaw)
        road = (rs > 0.5) & valid
    # fill ring gaps in intensity by normalised blur
    acc_f = cv2.filter2D(np.where(valid, acc, 0).astype(np.float32), -1, np.ones((3, 3), np.float32))
    cnt_f = cv2.filter2D(valid.astype(np.float32), -1, np.ones((3, 3), np.float32))
    inten = np.where(cnt_f > 0, acc_f / np.maximum(cnt_f, 1e-6), 0).astype(np.float32)
    return inten, road


def lane_response(gray, valid, k=0.7):
    """Thin bright marking response: white top-hat with ~k m kernel, contrast normalised, invalid=0."""
    g = gray.copy()
    if valid.any():
        g[~valid] = np.median(g[valid])
    ks = max(3, int(k / RES) | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (ks, ks))
    th = cv2.morphologyEx(g, cv2.MORPH_TOPHAT, kernel)
    loc_std = np.sqrt(np.maximum(cv2.blur(g * g, (31, 31)) - cv2.blur(g, (31, 31)) ** 2, 1e-3))
    resp = th / (loc_std + 1e-3)
    resp[~valid] = 0
    vm = cv2.erode(valid.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
    resp[~vm] = 0
    return np.clip(resp, 0, 4)


class GroundRaster:
    """Global road-surface height raster of the LiDAR map (low-percentile z per cell, holes filled, smoothed)."""

    def __init__(self, xyz, res=0.2):
        self.res = res
        self.x0 = float(xyz[:, 0].min()) - 1
        self.y0 = float(xyz[:, 1].min()) - 1
        nx = int((xyz[:, 0].max() - self.x0) / res) + 2
        ny = int((xyz[:, 1].max() - self.y0) / res) + 2
        z = np.full(nx * ny, np.inf, np.float32)
        step = 5_000_000
        for s in range(0, len(xyz), step):
            p = np.asarray(xyz[s:s + step])
            k = ((p[:, 0] - self.x0) / res).astype(np.int64) * ny + ((p[:, 1] - self.y0) / res).astype(np.int64)
            np.minimum.at(z, k, p[:, 2].astype(np.float32))
        z = z.reshape(nx, ny)
        valid = np.isfinite(z)
        zf = np.where(valid, z, 0).astype(np.float32)
        # fill holes by normalised box filtering at growing scales, then light smoothing
        out = zf.copy()
        filled = valid.copy()
        for k in [3, 7, 15, 31]:
            num = cv2.blur(np.where(filled, out, 0).astype(np.float32), (k, k))
            den = cv2.blur(filled.astype(np.float32), (k, k))
            fill = (~filled) & (den > 0.05)
            out[fill] = num[fill] / den[fill]
            filled |= fill
        self.z = cv2.GaussianBlur(out, (5, 5), 0)
        self.nx, self.ny = nx, ny

    def height(self, x, y):
        ix = np.clip(((x - self.x0) / self.res).astype(np.int64), 0, self.nx - 1)
        iy = np.clip(((y - self.y0) / self.res).astype(np.int64), 0, self.ny - 1)
        return self.z[ix, iy]


class ImageBEV3D(ImageBEV):
    """IPM onto the true map road surface with per-frame vehicle attitude.

    Camera pose per frame = road tilt under the vehicle (from the ground raster) x heading x dynamic pitch/roll
    (IMU high-pass, braking / bumps) x rig mounting; image pixels are sampled at the 3D road points.
    """

    def __init__(self, track_dir, ground, cam_height=1.5, mount=(0.0, 0.0, -1.5), dyn=None, cams=('f', 'fl', 'fr')):
        super().__init__(track_dir, cam_height, mount[0], mount[1], mount[2], cams)
        from scipy.spatial.transform import Rotation
        self.ground = ground
        self.h = cam_height
        self.Rm = Rotation.from_euler('ZYX', [mount[2], mount[0], mount[1]], degrees=True).as_matrix()
        self.dyn = dyn  # (N, 2) dynamic pitch, roll in degrees per frame, or None
        gx, gy = bev_grid()
        self.gx, self.gy = gx.ravel(), gy.ravel()

    def road_surface(self, x, y, c, s, wx, wy):
        """Smooth quadratic road surface z(u, v) (u fwd, v left) fitted robustly to the ground raster ahead."""
        uu, vv = np.meshgrid(np.arange(-4, X_MAX + 0.1, 1.0), np.arange(-8, 8.1, 1.0))
        uu, vv = uu.ravel(), vv.ravel()
        zz = self.ground.height(x + c * uu - s * vv, y + s * uu + c * vv)
        F = lambda u, v: np.stack([np.ones_like(u), u, v, u * u, v * v, u * v], 1)  # noqa: E731
        keep = np.ones(len(uu), bool)
        for _ in range(4):
            coef = np.linalg.lstsq(F(uu[keep], vv[keep]), zz[keep], rcond=None)[0]
            r = zz - F(uu, vv) @ coef
            keep = np.abs(r) < max(3 * np.median(np.abs(r[keep])) * 1.4826, 0.05)
        u = c * (wx - x) + s * (wy - y)
        v = -s * (wx - x) + c * (wy - y)
        return F(u, v) @ coef

    def render_pose(self, i, x, y, yaw):
        acc = np.zeros((H, W), np.float32)
        cnt = np.zeros((H, W), np.float32)
        for cname, (mx, my, v) in self.maps_pose(i, x, y, yaw).items():
            img = cv2.imread(os.path.join(self.track_dir, f'{self.track}_images', f'{self.track}_{cname}_images',
                                          self.names[cname][i]), cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            g, ok = self._sample(cname, i, img, mx, my, v)
            acc[ok] += g[ok]
            cnt[ok] += 1
        valid = cnt > 0
        acc[valid] /= cnt[valid]
        return acc, valid

    def _sample(self, cname, i, img, mx, my, v):
        bad = np.zeros(img.shape, np.uint8)
        if cname in self.masks:
            m = np.unpackbits(self.masks[cname][i], axis=1)[:, :img.shape[1] // 2]
            bad |= cv2.resize(m, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
        bad |= self.ego_bad(cname, img.shape)
        g = cv2.remap(img, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0).astype(np.float32)
        bm = cv2.remap(bad, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=1)
        return g, v & (bm == 0)

    def render_resp_pose(self, i, x, y, yaw, fn):
        """render_resp (per-camera feature response, blended afterwards, capture-time sync) on the 3D road."""
        acc = np.zeros((H, W), np.float32)
        cnt = np.zeros((H, W), np.float32)
        for cname, (mx, my, v) in self.maps_pose(i, x, y, yaw).items():
            img = cv2.imread(os.path.join(self.track_dir, f'{self.track}_images', f'{self.track}_{cname}_images',
                                          self.names[cname][i]), cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            g, ok = self._sample(cname, i, img, mx, my, v)
            if ok.sum() < 100:
                continue
            r_ = fn(g, ok)
            okr = cv2.erode(ok.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
            if self.tshift is not None:
                r_ = self._sync(cname, i, r_.astype(np.float32))
                okr = self._sync(cname, i, okr.astype(np.uint8), cv2.INTER_NEAREST) > 0
            acc[okr] += r_[okr]
            cnt[okr] += 1
        valid = cnt > 0
        acc[valid] /= cnt[valid]
        return acc, valid

    def maps_pose(self, i, x, y, yaw):
        """Per-camera remap tables sampling the image at the 3D map road points of the BEV grid at pose (x, y, yaw)."""
        from scipy.spatial.transform import Rotation
        c, s = np.cos(yaw), np.sin(yaw)
        wx = x + c * self.gx - s * self.gy
        wy = y + s * self.gx + c * self.gy
        wz = self.road_surface(x, y, c, s, wx, wy)
        # local road plane under the vehicle for the body attitude
        ox, oy = np.meshgrid(np.linspace(-4, 4, 9), np.linspace(-2, 2, 5))
        px = x + c * ox.ravel() - s * oy.ravel()
        py = y + s * ox.ravel() + c * oy.ravel()
        pz = self.ground.height(px, py)
        A = np.stack([px - x, py - y, np.ones_like(px)], 1)
        a, b, z0 = np.linalg.lstsq(A, pz, rcond=None)[0]
        n = np.array([-a, -b, 1.0])
        n /= np.linalg.norm(n)
        from register import rot_z_to
        R = rot_z_to(n) @ Rotation.from_euler('z', yaw).as_matrix()
        if self.dyn is not None:
            R = R @ Rotation.from_euler('YX', [self.dyn[i, 0], self.dyn[i, 1]], degrees=True).as_matrix()
        cam = np.array([x, y, z0]) + R @ np.array([0, 0, self.h])
        q = (np.stack([wx, wy, wz], 1) - cam) @ R  # vehicle frame
        q = q @ self.Rm
        maps = {}
        for cname in self.cams:
            uv, valid = self.C.project(cname, q)
            mx = uv[:, 0].reshape(H, W).astype(np.float32)
            my = uv[:, 1].reshape(H, W).astype(np.float32)
            v = valid.reshape(H, W)
            mx[~v] = -1
            my[~v] = -1
            maps[cname] = (mx, my, v)
        return maps


def map_curb(xyz, pose_xy, yaw, step_min=0.08, step_max=0.40, win=0.4):
    """Curb / median-island edges of the map in the vehicle BEV frame: cells where the local ground height
    (lowest points per 0.1 m cell) jumps by step_min..step_max within win metres (kerbs, island edges);
    larger jumps (walls, vehicles, poles) are rejected. Returns a float raster (step height, smoothed)."""
    c, s = np.cos(yaw), np.sin(yaw)
    d = xyz[:, :2] - pose_xy
    x = c * d[:, 0] + s * d[:, 1]
    y = -s * d[:, 0] + c * d[:, 1]
    z = xyz[:, 2].astype(np.float32)
    r = ((X_MAX - x) / RES).astype(np.int64)
    col = ((Y_MAX - y) / RES).astype(np.int64)
    ok = (r >= 0) & (r < H) & (col >= 0) & (col < W)
    hmin = np.full(H * W, np.inf, np.float32)
    np.minimum.at(hmin, r[ok] * W + col[ok], z[ok])
    hmin = hmin.reshape(H, W)
    valid = np.isfinite(hmin)
    base = np.median(hmin[valid]) if valid.any() else 0.0
    h = np.where(valid, hmin, base).astype(np.float32)
    # fill isolated holes with the local minimum of valid neighbours (ground is the lowest surface)
    filled = cv2.erode(np.where(valid, h, 1e4).astype(np.float32), np.ones((3, 3), np.uint8))
    h = np.where(valid, h, np.where(filled < 1e3, filled, base)).astype(np.float32)
    v3 = cv2.dilate(valid.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    k = max(3, int(win / RES) | 1)
    ker = np.ones((k, k), np.uint8)
    step = cv2.dilate(h, ker) - cv2.erode(h, ker)
    tall = cv2.dilate((h - cv2.erode(h, np.ones((15, 15), np.uint8)) > step_max).astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
    curb = ((step > step_min) & (step < step_max) & v3 & ~tall).astype(np.uint8)  # no edges of vehicles / scooters
    curb = line_open(curb)  # keep long kerb lines, drop vegetation / vehicle blobs
    out = np.where(curb > 0, np.minimum(step, step_max), 0).astype(np.float32)
    return cv2.GaussianBlur(out, (0, 0), 1.0)


def line_open(img, length=2.0):
    """Morphological opening with along-road and across-road line elements (keeps long straight structures)."""
    L = max(3, int(length / RES))
    a = cv2.morphologyEx(img, cv2.MORPH_OPEN, np.ones((L, 1), np.uint8))   # along the road (BEV rows)
    b = cv2.morphologyEx(img, cv2.MORPH_OPEN, np.ones((1, L), np.uint8))   # across the road (intersections)
    return np.maximum(a, b)


def edge_response(gray, valid):
    """Image BEV edge strength (kerb faces, shadows, material boundaries), contrast-normalised, invalid=0."""
    g = gray.copy()
    if valid.any():
        g[~valid] = np.median(g[valid])
    g = cv2.GaussianBlur(g, (0, 0), 1.0)
    gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
    mag = np.sqrt(gx * gx + gy * gy)
    loc = cv2.blur(mag, (31, 31)) + 1e-3
    resp = np.clip(mag / loc, 0, 4).astype(np.float32)
    resp = line_open(resp)  # projected tall objects make radial streaks; kerbs are along/across the road
    vm = cv2.erode(valid.astype(np.uint8), np.ones((7, 7), np.uint8)) > 0
    resp[~vm] = 0
    return resp


def map_road_region(xyz, pose_xy, yaw, step_max=0.08, win=0.4):
    """Drivable road surface around the vehicle bounded by kerbs.

    Lowest map point per 0.1 m cell, detrended by a robust plane (the map is tilted ~10 deg w.r.t. the vehicle
    and roads have crossfall), median-filtered; flood fill from under the vehicle over cells whose local height
    step within `win` is below step_max, so kerbs and island edges stop the fill. Returns float 0..1."""
    c, s = np.cos(yaw), np.sin(yaw)
    d = xyz[:, :2] - pose_xy
    x = c * d[:, 0] + s * d[:, 1]
    y = -s * d[:, 0] + c * d[:, 1]
    z = xyz[:, 2]
    r = ((X_MAX - x) / RES).astype(np.int64)
    col = ((Y_MAX - y) / RES).astype(np.int64)
    ok = (r >= 0) & (r < H) & (col >= 0) & (col < W)
    hmin = np.full(H * W, np.inf)
    np.minimum.at(hmin, r[ok] * W + col[ok], z[ok])
    hmin = hmin.reshape(H, W)
    valid = np.isfinite(hmin)
    if valid.sum() < 200:
        return np.zeros((H, W), np.float32)
    rr, cc = np.nonzero(valid)
    xs, ys, zs = X_MAX - rr * RES, Y_MAX - cc * RES, hmin[valid]
    near = (np.abs(ys) < 4) & (xs > NEAR - 3) & (xs < NEAR + 12)  # road under / ahead of the vehicle
    if near.sum() < 50:
        near = np.ones_like(zs, bool)
    keep = near.copy()
    for _ in range(4):  # robust plane on the lowest surface near the vehicle
        A = np.stack([xs[keep], ys[keep], np.ones(keep.sum())], 1)
        coef = np.linalg.lstsq(A, zs[keep], rcond=None)[0]
        res = zs - (coef[0] * xs + coef[1] * ys + coef[2])
        keep = near & (np.abs(res) < max(3 * np.median(np.abs(res[keep])), 0.05))
    gx, gy = np.meshgrid(Y_MAX - np.arange(W) * RES, X_MAX - np.arange(H) * RES)
    plane = coef[0] * gy + coef[1] * gx + coef[2]
    hd = np.where(valid, hmin - plane, np.nan).astype(np.float32)
    big = np.where(valid, hd, 1e4).astype(np.float32)
    fill = cv2.erode(big, np.ones((3, 3), np.uint8))
    hd = np.where(valid, hd, np.where(fill < 1e3, fill, np.nan)).astype(np.float32)
    known = np.isfinite(hd)
    hk = np.where(known, hd, 0).astype(np.float32)
    hk = cv2.medianBlur(hk, 5)
    k = max(3, int(win / RES) | 1)
    ker = np.ones((k, k), np.uint8)
    step = cv2.dilate(np.where(known, hk, -1e4).astype(np.float32), ker) - cv2.erode(np.where(known, hk, 1e4).astype(np.float32), ker)
    free = (known & (step < step_max) & (np.abs(hk) < 0.6)).astype(np.uint8)
    seeds = [(int((X_MAX - xx) / RES), int(Y_MAX / RES) + dc) for xx in np.arange(NEAR + 0.5, NEAR + 8, 0.5) for dc in (-5, 0, 5)]
    mask = np.zeros((H + 2, W + 2), np.uint8)
    region = free.copy()
    for (r0, c0) in seeds:
        if 0 <= r0 < H and 0 <= c0 < W and region[r0, c0] == 1:
            cv2.floodFill(region, mask, (c0, r0), 2, loDiff=0, upDiff=0, flags=4)
    road = (region == 2).astype(np.float32)
    road = cv2.morphologyEx(road, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    return cv2.GaussianBlur(road, (0, 0), 1.5)


def map_road_scan(xyz, pose_xy, yaw, curb_h=0.07, curb_run=0.3, gap_run=1.5, drop_h=0.25, smooth_rows=11, obst_h=0.35,
                  obst_run=6.0):
    """Road surface bounded by kerbs, found by scanning every BEV row outwards from the vehicle centre line.

    Height is the lowest map point per cell, detrended by a robust plane fitted near the vehicle (map tilt, road
    crossfall) and median-filtered. Scanning left and right, the boundary is the first place where the surface is
    curb_h above the road for curb_run metres (kerb, island, sidewalk), drops by drop_h, or has no map data for
    gap_run metres. Boundaries are median-smoothed along the road. Returns float 0..1."""
    c, s = np.cos(yaw), np.sin(yaw)
    d = xyz[:, :2] - pose_xy
    x = c * d[:, 0] + s * d[:, 1]
    y = -s * d[:, 0] + c * d[:, 1]
    z = xyz[:, 2]
    r = ((X_MAX - x) / RES).astype(np.int64)
    col = ((Y_MAX - y) / RES).astype(np.int64)
    ok = (r >= 0) & (r < H) & (col >= 0) & (col < W)
    hmin = np.full(H * W, np.inf)
    np.minimum.at(hmin, r[ok] * W + col[ok], z[ok])
    hmin = hmin.reshape(H, W)
    valid = np.isfinite(hmin)
    if valid.sum() < 200:
        return np.zeros((H, W), np.float32)
    rr, cc = np.nonzero(valid)
    xs, ys, zs = X_MAX - rr * RES, Y_MAX - cc * RES, hmin[valid]
    near = (np.abs(ys) < 3) & (xs > NEAR - 3) & (xs < NEAR + 10)
    if near.sum() < 50:
        near = np.ones_like(zs, bool)
    keep = near.copy()
    for _ in range(4):
        A = np.stack([xs[keep], ys[keep], np.ones(keep.sum())], 1)
        coef = np.linalg.lstsq(A, zs[keep], rcond=None)[0]
        res = zs - (coef[0] * xs + coef[1] * ys + coef[2])
        keep = near & (np.abs(res) < max(3 * np.median(np.abs(res[keep])), 0.04))
    gy, gx = np.meshgrid(Y_MAX - np.arange(W) * RES, X_MAX - np.arange(H) * RES)
    hd = np.where(valid, hmin - (coef[0] * gx + coef[1] * gy + coef[2]), 0).astype(np.float32)
    hd = cv2.medianBlur(hd, 5)
    known = cv2.dilate(valid.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    nc, ng = max(1, int(curb_run / RES)), max(1, int(gap_run / RES))
    # kerbs / islands are 7-35 cm high; taller blobs on the road are vehicles or scooters left in the map when it
    # was built: they are skipped (not a boundary) unless the obstacle is very wide (obst_run, e.g. a wall/building)
    obst = known & (hd >= obst_h)
    stop_curb = known & (hd > curb_h) & (hd < obst_h)
    stop_drop = known & (hd < -drop_h)
    no = max(1, int(obst_run / RES))
    wide_obst = cv2.filter2D(obst.astype(np.float32), -1, np.ones((1, no), np.float32), anchor=(0, 0),
                             borderType=cv2.BORDER_CONSTANT) >= no - 0.5
    known_scan = known & ~(obst & ~wide_obst)  # narrow obstacles count as unknown
    c0 = int(Y_MAX / RES)

    def first_stop(side):  # side: (H, n) cells walking outwards from the centre
        sc, sd, kn = side
        run_c = cv2.filter2D(sc.astype(np.float32), -1, np.ones((1, nc), np.float32), anchor=(0, 0),
                             borderType=cv2.BORDER_CONSTANT) >= nc - 0.5
        run_g = cv2.filter2D((~kn).astype(np.float32), -1, np.ones((1, ng), np.float32), anchor=(0, 0),
                             borderType=cv2.BORDER_CONSTANT) >= ng - 0.5
        stop = run_c | sd | run_g
        idx = np.where(stop.any(1), stop.argmax(1), stop.shape[1])
        return idx

    stop_curb = stop_curb | (obst & wide_obst)
    left = first_stop((stop_curb[:, c0::-1], stop_drop[:, c0::-1], known_scan[:, c0::-1]))
    right = first_stop((stop_curb[:, c0:], stop_drop[:, c0:], known_scan[:, c0:]))
    from scipy.ndimage import median_filter
    left = median_filter(left, smooth_rows, mode='nearest')
    right = median_filter(right, smooth_rows, mode='nearest')
    cols = np.arange(W)[None, :]
    road = ((cols >= (c0 - left)[:, None]) & (cols <= (c0 + right)[:, None])).astype(np.float32)
    road[~known.any(1)] = 0
    return cv2.GaussianBlur(road, (0, 0), 1.5)


def map_road_level(xyz, pose_xy, yaw, raise_h=0.08, win=4.0, obst_h=0.35):
    """Road surface defined like the image segmentation 'road': every cell at road level, whichever carriageway.

    Lowest map point per 0.1 m cell (holes filled with local minima, median-filtered). A cell is raised (sidewalk,
    median island, traffic island) if it is more than raise_h above the lowest surface within `win` metres; cells
    much higher (>= obst_h: vehicles / scooters left in the map, walls) are unknown. Returns (road 0..1, known)
    so unknown cells can be treated as neutral in the correlation."""
    c, s = np.cos(yaw), np.sin(yaw)
    d = xyz[:, :2] - pose_xy
    x = c * d[:, 0] + s * d[:, 1]
    y = -s * d[:, 0] + c * d[:, 1]
    z = xyz[:, 2]
    r = ((X_MAX - x) / RES).astype(np.int64)
    col = ((Y_MAX - y) / RES).astype(np.int64)
    ok = (r >= 0) & (r < H) & (col >= 0) & (col < W)
    hmin = np.full(H * W, np.inf)
    np.minimum.at(hmin, r[ok] * W + col[ok], z[ok])
    hmin = hmin.reshape(H, W).astype(np.float32)
    valid = np.isfinite(hmin)
    if valid.sum() < 200:
        return np.zeros((H, W), np.float32), np.zeros((H, W), bool)
    # detrend: the map is tilted ~10 deg w.r.t. the vehicle (and roads have crossfall / grade)
    rr, cc = np.nonzero(valid)
    xs, ys, zs = X_MAX - rr * RES, Y_MAX - cc * RES, hmin[valid].astype(np.float64)
    keep = zs < np.percentile(zs, 60)
    for _ in range(5):
        A = np.stack([xs[keep], ys[keep], np.ones(keep.sum())], 1)
        coef = np.linalg.lstsq(A, zs[keep], rcond=None)[0]
        res = zs - A.dot(coef) if False else zs - (coef[0] * xs + coef[1] * ys + coef[2])
        keep = np.abs(res) < max(2.5 * np.median(np.abs(res[keep])), 0.05)
    gy, gx = np.meshgrid(Y_MAX - np.arange(W) * RES, X_MAX - np.arange(H) * RES)
    hmin = np.where(valid, hmin - (coef[0] * gx + coef[1] * gy + coef[2]), np.inf).astype(np.float32)
    big = np.where(valid, hmin, 1e4).astype(np.float32)
    fill = cv2.erode(big, np.ones((5, 5), np.uint8))
    h = np.where(valid, hmin, fill)
    known = h < 1e3
    h = np.where(known, h, 0).astype(np.float32)
    h = cv2.medianBlur(h, 5)
    k = max(3, int(win / RES) | 1)
    low = cv2.erode(np.where(known, h, 1e4).astype(np.float32), np.ones((k, k), np.uint8))   # local lowest surface
    low = cv2.blur(np.minimum(low, 1e3), (k // 2 | 1, k // 2 | 1))                             # smooth the reference
    above = h - low
    road = (above < raise_h).astype(np.float32)
    unknown = (above >= obst_h) | ~known
    road = cv2.GaussianBlur(road, (0, 0), 1.0)
    return road, ~unknown


ROAD_SLOPE = os.environ.get('SDC_ROAD_SLOPE', 'smooth')  # 'smooth': artefact-free slope (see map_road_smooth)
ROAD_LEVEL_TOL = 0.0   # v3 (ramp-proof road level) when > 0, e.g. 0.08
ROAD_FILL_M2 = 0.0     # v3 small-hole filling when > 0, e.g. 3.0


def map_road_smooth(xyz, pose_xy, yaw, slope_max=0.25, base=0.3, connect=True, fill_hole_m2=None, standing_h=0.0,
                    level_tol=None):
    """Road = smooth ground: cells whose local height slope (over `base` metres) is below slope_max after removing
    the typical slope of the area (the map is tilted ~10 deg w.r.t. the vehicle). Gentle grades, crests and
    crossfall pass; kerbs and island edges (10-20 cm drops) are steep and cut the region. With `connect`, only the
    smooth region connected to the cells ahead of the vehicle is kept. Returns (road 0..1, known)."""
    level_tol = ROAD_LEVEL_TOL if level_tol is None else level_tol
    fill_hole_m2 = ROAD_FILL_M2 if fill_hole_m2 is None else fill_hole_m2
    c, s = np.cos(yaw), np.sin(yaw)
    d = xyz[:, :2] - pose_xy
    x = c * d[:, 0] + s * d[:, 1]
    y = -s * d[:, 0] + c * d[:, 1]
    z = xyz[:, 2]
    r = ((X_MAX - x) / RES).astype(np.int64)
    col = ((Y_MAX - y) / RES).astype(np.int64)
    ok = (r >= 0) & (r < H) & (col >= 0) & (col < W)
    hmin = np.full(H * W, np.inf)
    np.minimum.at(hmin, r[ok] * W + col[ok], z[ok])
    hmin = hmin.reshape(H, W).astype(np.float32)
    valid = np.isfinite(hmin)
    if valid.sum() < 200:
        return np.zeros((H, W), np.float32), np.zeros((H, W), bool)
    h = np.where(valid, hmin, 1e4).astype(np.float32)
    for k in (3, 5):  # fill small scan gaps with local minima
        h = np.where(h > 1e3, cv2.erode(h, np.ones((k, k), np.uint8)), h)
    known = h < 1e3
    h = np.where(known, h, np.median(hmin[valid])).astype(np.float32)
    h = cv2.medianBlur(h, 5)
    b = max(1, int(round(base / RES)))
    if ROAD_SLOPE == 'smooth':
        # gradient of a known-weighted smoothed surface: one missing cell only affects its neighbourhood
        # ('roll' sampled h at +-b cells, so a single unknown / spiky cell punched 5 holes in a '+' pattern)
        kf = known.astype(np.float32)
        sig = b / 2.0
        hs = cv2.GaussianBlur(np.where(known, h, 0).astype(np.float32), (0, 0), sig) / \
            np.maximum(cv2.GaussianBlur(kf, (0, 0), sig), 1e-3)
        gx = cv2.Sobel(hs, cv2.CV_32F, 1, 0, ksize=3) / (8 * RES)
        gy = cv2.Sobel(hs, cv2.CV_32F, 0, 1, ksize=3) / (8 * RES)
        kn2 = known & (cv2.blur(kf, (2 * b + 1, 2 * b + 1)) > 0.6)
    else:
        gx = (np.roll(h, -b, 1) - np.roll(h, b, 1)) / (2 * b * RES)
        gy = (np.roll(h, -b, 0) - np.roll(h, b, 0)) / (2 * b * RES)
        kn2 = known & np.roll(known, b, 0) & np.roll(known, -b, 0) & np.roll(known, b, 1) & np.roll(known, -b, 1)
    gx0, gy0 = np.median(gx[kn2]), np.median(gy[kn2])  # typical slope (map tilt / crossfall)
    slope = np.hypot(gx - gx0, gy - gy0)
    smooth = kn2 & (slope < slope_max)
    if level_tol > 0:
        # road level: a smooth quadratic surface fitted to the smooth cells near / ahead of the vehicle (robust,
        # iteratively dropping cells above it). Sidewalks are ~15 cm higher, so cells more than level_tol above the
        # road level are excluded even when a gentle wheelchair ramp connects them to the road.
        gyy, gxx = np.meshgrid(Y_MAX - np.arange(W) * RES, X_MAX - np.arange(H) * RES)
        F = lambda u, v: np.stack([np.ones_like(u), u, v, u * u, v * v, u * v], -1)  # noqa: E731
        sel = smooth & (np.abs(gyy) < 6) & (gxx > NEAR - 3) & (gxx < NEAR + 15)
        if sel.sum() > 200:
            u0, v0, z0 = gxx[sel], gyy[sel], h[sel]
            keep = np.ones(len(z0), bool)
            for _ in range(5):
                coef = np.linalg.lstsq(F(u0[keep], v0[keep]), z0[keep], rcond=None)[0]
                res = z0 - F(u0, v0) @ coef
                keep = res < max(2.5 * np.median(np.abs(res[keep])), 0.03)   # drop raised cells (sidewalk)
            level = F(gxx, gyy) @ coef
            smooth &= (h - level) < level_tol
    if connect:
        lab_n, lab = cv2.connectedComponents(smooth.astype(np.uint8), connectivity=4)
        seeds = [(int((X_MAX - xx) / RES), int(Y_MAX / RES) + dc) for xx in np.arange(NEAR + 0.5, NEAR + 8, 0.5)
                 for dc in (-8, 0, 8)]
        ids = {lab[r0, c0] for r0, c0 in seeds if 0 <= r0 < H and 0 <= c0 < W and lab[r0, c0] > 0}
        smooth = np.isin(lab, list(ids)) if ids else smooth
    road = cv2.morphologyEx(smooth.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)) > 0
    known_out = kn2.copy()
    if standing_h > 0:
        # objects standing in the map (vehicles / scooters parked when it was built, planters...): lowest point well
        # above the local ground. They are UNKNOWN, not non-road: if the object is gone today the image shows road,
        # and if a car parks there today the image segmentation marks it as unknown -> neither case biases the match
        g = local_ground(np.where(valid, hmin, np.inf).astype(np.float32), valid)
        standing = valid & (hmin - g > standing_h)
        standing = cv2.dilate(standing.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
        known_out &= ~standing
    if fill_hole_m2 > 0:
        # small holes inside the road: patched pavement, manholes, drains, scan noise, multi-pass seams
        from scipy import ndimage
        holes = ndimage.binary_fill_holes(road) & ~road
        lab_h, nh = ndimage.label(holes)
        if nh:
            sizes = ndimage.sum(holes, lab_h, np.arange(1, nh + 1)) * RES * RES
            small = np.isin(lab_h, np.flatnonzero(sizes <= fill_hole_m2) + 1)
            road = road | small
    road = road.astype(np.float32)
    return cv2.GaussianBlur(road, (0, 0), 1.0), known_out


class RoadProbBEV:
    """IPM of SegFormer road probability (quarter-res arrays) using an ImageBEV's projection maps."""

    def __init__(self, bev, subdir='road', mode='road'):
        """mode: 'road' = P(road + vehicles); 'road_minus_side' = (P(road) - P(sidewalk) + 1) / 2, which pushes the
        kerb boundary (road <-> sidewalk) apart in the image probability."""
        self.bev = bev
        d = os.path.join(bev.track_dir, subdir)
        z = {c: np.load(os.path.join(d, f'{c}.npz')) for c in bev.cams}
        self.prob = {c: z[c]['road'] for c in bev.cams}
        # road users from the segmentation itself (if saved): unknown, neither road nor non-road
        self.obj = {c: z[c]['obj'] for c in bev.cams} if 'obj' in z['f'] else None
        self.side = {c: z[c]['side'] for c in bev.cams} if mode == 'road_minus_side' and 'side' in z['f'] else None
        self.use_masks = True
        self.colour = mode == 'seg_colour'  # average with a per-frame asphalt colour model (Lab) where both exist

    def colour_prob(self, i, c, img_shape_full):
        """Per-frame asphalt colour model: Lab statistics of the band just above the hood (front camera, no road
        users, paint dropped); chroma tight, lightness loose (shadows, exposure). Returns full-res probability."""
        b = self.bev
        if c == 'f' or not hasattr(self, '_ref') or self._ref[0] != i:
            img = cv2.imread(os.path.join(b.track_dir, f'{b.track}_images', f'{b.track}_f_images', b.names['f'][i]))
            lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB).astype(np.float32)
            mk = cv2.resize(np.unpackbits(b.masks['f'][i], axis=1)[:, :img.shape[1] // 2], (img.shape[1], img.shape[0]),
                            interpolation=cv2.INTER_NEAREST) > 0 if 'f' in b.masks else np.zeros(img.shape[:2], bool)
            top = int(np.argmax(b.hood, axis=0).min()) if b.hood is not None else img.shape[0] - 120
            rows = np.arange(img.shape[0])[:, None]
            cols = np.arange(img.shape[1])[None, :]
            band = (rows > top - 90) & (rows < top - 15) & (np.abs(cols - img.shape[1] // 2) < 260) & ~mk
            if b.hood is not None:
                band &= ~b.hood
            sm = lab[band]
            sm = sm[sm[:, 0] < np.percentile(sm[:, 0], 85)]
            med = np.median(sm, 0)
            sd = np.maximum(1.4826 * np.median(np.abs(sm - med), 0), [6, 2, 2])
            self._ref = (i, med, sd)
        _, med, sd = self._ref
        img = cv2.imread(os.path.join(b.track_dir, f'{b.track}_images', f'{b.track}_{c}_images', b.names[c][i]))
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB).astype(np.float32)
        d2 = (((lab[..., 1] - med[1]) / (3 * sd[1])) ** 2 + ((lab[..., 2] - med[2]) / (3 * sd[2])) ** 2
              + ((lab[..., 0] - med[0]) / (6 * sd[0])) ** 2)
        prob = cv2.GaussianBlur(np.exp(-0.5 * d2).astype(np.float32), (0, 0), 2)
        # keep only asphalt-coloured pixels connected to the road just in front of / beside the car: grey sky,
        # building walls and signs have similar colour but are not connected to the road surface
        hh, ww = prob.shape
        cand = (prob > 0.35).astype(np.uint8)
        if c in b.masks:  # road users break the road into pieces; let the road connect through them
            mk = cv2.resize(np.unpackbits(b.masks[c][i], axis=1)[:, :ww // 2], (ww, hh), interpolation=cv2.INTER_NEAREST)
            cand = np.maximum(cand, mk)
        if c == 'f' and b.hood is not None:
            cand[b.hood] = 0
        n, lab_ = cv2.connectedComponents(cv2.morphologyEx(cand, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)), connectivity=4)
        if c == 'f':
            top = int(np.argmax(b.hood, axis=0).min()) if b.hood is not None else hh - 120
            seed = lab_[max(top - 60, 0):max(top - 10, 1), ww // 2 - 200:ww // 2 + 200]
        else:  # side cameras: the road next to the car is at the bottom, towards the vehicle side
            seed = lab_[hh - 120:hh - 20, :]
        ids = np.unique(seed[seed > 0])
        if len(ids):
            counts = np.bincount(seed[seed > 0].ravel())
            ids = [k for k in ids if counts[k] > 0.05 * (seed > 0).sum()]
            prob = prob * np.isin(lab_, ids)
        return prob

    def render(self, i):
        b = self.bev
        if b.dyn is not None:
            h, p, r, y = b.calib
            b.set_calib(h, p + b.dyn[i, 0], r + b.dyn[i, 1], y)
        acc = np.zeros((H, W), np.float32)
        cnt = np.zeros((H, W), np.float32)
        for c in b.cams:
            pr = self.prob[c][i].astype(np.float32) / 255.0
            if self.side is not None:
                pr = np.clip((pr - self.side[c][i].astype(np.float32) / 255.0 + 1) / 2, 0, 1)
            hq, wq = pr.shape
            bad = np.zeros((hq * 4, wq * 4), np.uint8)
            # YOLO road users are left out as *unknown* (neutral in the NCC, not "non-road"): projected car bodies
            # would otherwise leak "road" beyond the kerb, since IPM assumes every pixel lies on the ground
            if self.use_masks and c in b.masks:
                m = np.unpackbits(b.masks[c][i], axis=1)[:, :wq * 2]
                bad |= cv2.resize(m, (wq * 4, hq * 4), interpolation=cv2.INTER_NEAREST)
            if self.obj is not None:
                ob = cv2.dilate((self.obj[c][i] > 100).astype(np.uint8), np.ones((5, 5), np.uint8))
                bad |= cv2.resize(ob, (wq * 4, hq * 4), interpolation=cv2.INTER_NEAREST)
            bad |= b.ego_bad(c, bad.shape)
            mx, my, v = b.maps[c]
            g = cv2.remap(pr, mx / 4.0, my / 4.0, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
            if self.colour:
                cpb = cv2.remap(self.colour_prob(i, c, None), mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
                g = 0.5 * (g + cpb)
            bm = cv2.remap(bad, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=1)
            ok = v & (bm == 0)
            if b.tshift is not None:
                g = b._sync(c, i, g.astype(np.float32))
                ok = b._sync(c, i, ok.astype(np.uint8), cv2.INTER_NEAREST) > 0
            acc[ok] += g[ok]
            cnt[ok] += 1
        valid = cnt > 0
        acc[valid] /= cnt[valid]
        return acc, valid
