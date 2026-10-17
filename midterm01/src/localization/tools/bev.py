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
X_MIN, X_MAX = 3.0, 30.0
Y_MAX = 15.0
H = int((X_MAX - X_MIN) / RES)
W = int(2 * Y_MAX / RES)


def bev_grid():
    r, c = np.mgrid[0:H, 0:W]
    x = X_MAX - (r + 0.5) * RES
    y = Y_MAX - (c + 0.5) * RES
    return x, y


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
        self.masks = {}
        mdir = os.path.join(track_dir, 'masks')
        for c in cams:
            f = os.path.join(mdir, f'{c}.npz')
            if os.path.exists(f):
                self.masks[c] = np.load(f)['masks']
        self.calib = (cam_height, pitch_deg, roll_deg, yaw_deg)
        self.dyn = None  # optional (N, 2) per-frame dynamic pitch/roll offsets in degrees
        self.set_calib(cam_height, pitch_deg, roll_deg, yaw_deg)

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
            if c == 'f' and self.hood is not None and self.hood.shape == img.shape:
                bad |= self.hood.astype(np.uint8)
            mx, my, v = self.maps[c]
            g = cv2.remap(img, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0).astype(np.float32)
            b = cv2.remap(bad, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=1)
            ok = v & (b == 0)
            acc[ok] += g[ok]
            cnt[ok] += 1
        valid = cnt > 0
        acc[valid] /= cnt[valid]
        return acc, valid


def map_bev(xyz, intensity, pose_xy, yaw, plane_tol=0.25, rough_max=0.08):
    """Road-surface intensity BEV of map points around pose (vehicle BEV frame).

    Road = cells whose lowest points lie on the local road plane (fitted under the vehicle) within plane_tol
    and are flat (z spread of near-ground points < rough_max); vegetation and kerbside clutter are rejected.
    Returns (intensity raster, road mask).
    """
    c, s = np.cos(yaw), np.sin(yaw)
    d = xyz[:, :2] - pose_xy
    x = c * d[:, 0] + s * d[:, 1]
    y = -s * d[:, 0] + c * d[:, 1]
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
    r = ((X_MAX - x) / RES).astype(np.int64)
    col = ((Y_MAX - y) / RES).astype(np.int64)
    ok = (r >= 0) & (r < H) & (col >= 0) & (col < W) & (np.abs(hrel) < plane_tol + 0.3)
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
        acc = np.zeros((H, W), np.float32)
        cnt = np.zeros((H, W), np.float32)
        for cname in self.cams:
            uv, valid = self.C.project(cname, q)
            mx = uv[:, 0].reshape(H, W).astype(np.float32)
            my = uv[:, 1].reshape(H, W).astype(np.float32)
            v = valid.reshape(H, W)
            mx[~v] = -1
            my[~v] = -1
            img = cv2.imread(os.path.join(self.track_dir, f'{self.track}_images', f'{self.track}_{cname}_images',
                                          self.names[cname][i]), cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            bad = np.zeros(img.shape, np.uint8)
            if cname in self.masks:
                m = np.unpackbits(self.masks[cname][i], axis=1)[:, :img.shape[1] // 2]
                bad |= cv2.resize(m, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
            if cname == 'f' and self.hood is not None and self.hood.shape == img.shape:
                bad |= self.hood.astype(np.uint8)
            g = cv2.remap(img, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0).astype(np.float32)
            bm = cv2.remap(bad, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=1)
            ok = v & (bm == 0)
            acc[ok] += g[ok]
            cnt[ok] += 1
        valid = cnt > 0
        acc[valid] /= cnt[valid]
        return acc, valid
