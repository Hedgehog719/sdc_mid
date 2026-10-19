#!/usr/bin/env python3
"""Map-to-image localization by BEV road-marking matching.

For every frame the LiDAR map's road-surface intensity is rendered as a lane-marking raster at the prior pose,
the three fisheye images are inverse-perspective-mapped onto the road plane (calibrated intrinsics + rig
mounting from calib_bev.py, YOLO road users and the ego hood masked), and the image marking raster is matched
to the map raster over yaw (rotation search) and x/y (normalised cross-correlation). This uses the map's
reflectivity features in the camera image instead of the noisy image-derived point clouds.

Output npz (per frame): idx, ts, prior (x, y, yaw), meas (x, y, yaw), peak (NCC), sharp (peak - 2nd peak),
shift (fwd, lat, dyaw) in the prior vehicle frame.
"""
import argparse
import multiprocessing as mp
import os
import sys
import time

import cv2
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(__file__))
from bev import (H, RES, W, X_MAX, Y_MAX, GroundRaster, ImageBEV, ImageBEV3D, edge_response, lane_response,  # noqa: E402
                 RoadProbBEV, map_bev, map_curb, map_road_region, map_road_scan, map_road_smooth)
from common import TiledMap, ply_frames, wrap  # noqa: E402

A = None


def map_files(track_dir):
    """map / intensity files; SDC_MAP_TAG=<tag> selects map_<tag>.npy (e.g. clean_map.py output)."""
    tag = os.environ.get('SDC_MAP_TAG', '')
    sfx = f'_{tag}' if tag else ''
    return os.path.join(track_dir, f'map{sfx}.npy'), os.path.join(track_dir, f'map_intensity{sfx}.npy')


def load_map(track_dir):
    fm, fi = map_files(track_dir)
    M = TiledMap(fm, keep_order=True)
    M.inten = np.load(fi, mmap_mode='r')[M.order]
    del M.order
    return M


def rotate_raster(img, dyaw):
    """Rotate a vehicle-BEV raster by dyaw (rad, CCW) about the vehicle origin (x=0, y=0)."""
    r0, c0 = X_MAX / RES, Y_MAX / RES  # pixel coords of origin (row, col)
    # pixel (c, r) <-> (x, y): x = X_MAX - r*RES, y = Y_MAX - c*RES ; rotating points by +dyaw in xy
    ang = np.degrees(dyaw)
    Mrot = cv2.getRotationMatrix2D((c0, r0), -ang, 1.0)
    return cv2.warpAffine(img, Mrot, (W, H), flags=cv2.INTER_LINEAR, borderValue=0)


def pix_affine(dx, dy, dpsi):
    """2x3 pixel affine mapping a raster of frame j into frame i, given j's pose in i's frame (dx, dy, dpsi)."""
    Ap = np.array([[0, -RES, X_MAX], [-RES, 0, Y_MAX], [0, 0, 1.0]])  # (c, r, 1) -> (x, y, 1)
    c, s = np.cos(dpsi), np.sin(dpsi)
    Tm = np.array([[c, -s, dx], [s, c, dy], [0, 0, 1.0]])
    return (np.linalg.inv(Ap) @ Tm @ Ap)[:2]


def neighbour_frames(i):
    """Frames accumulated with frame i: by path distance (+-accum_dist m, one per accum_ds m) so that slow driving
    (Track1 start, 1-2.5 m/s) still sees as much road as normal driving; otherwise a fixed frame window."""
    if A.accum_dist <= 0:
        return [j for j in range(i - A.accum, i + A.accum + 1, A.accum_step) if 0 <= j < len(PRIOR_REL)]
    s = PATH_S
    lo = np.searchsorted(s, s[i] - A.accum_dist)
    hi = np.searchsorted(s, s[i] + A.accum_dist, side='right')
    cand = np.arange(lo, hi)
    if len(cand) == 0:
        return [i]
    # one frame per accum_ds metres of travel (and at least every accum_step frames when nearly stationary)
    key = np.floor((s[cand] - s[i]) / A.accum_ds).astype(int)
    _, first = np.unique(key, return_index=True)
    sel = set(cand[first].tolist()) | {i}
    if len(sel) < 5:  # nearly stationary: fall back to the time window
        sel |= {j for j in range(i - A.accum, i + A.accum + 1, A.accum_step) if 0 <= j < len(PRIOR_REL)}
    return sorted(sel)


MARKS = None


def accumulated_response(i, road_d):
    """Image lane response of frame i, averaged with neighbours warped by the prior's relative motion."""
    xi, yi, yawi = PRIOR_REL[i]
    acc = np.zeros((H, W), np.float32)
    cnt = np.zeros((H, W), np.float32)
    for j in neighbour_frames(i):
        if j < 0 or j >= len(PRIOR_REL):
            continue
        if A.feature == 'road':
            r, v = ROADBEV.render(j)
        elif A.percam and A.feature == 'lane' and A.ground3d:
            r, v = BEV.render_resp_pose(j, *PRIOR_REL[j], lane_response)
        elif A.percam and A.feature == 'lane':
            r, v = (BEV.render_resp_color if A.color else BEV.render_resp)(j, lane_response)
        else:
            g, v = BEV.render_pose(j, *PRIOR_REL[j]) if A.ground3d else BEV.render(j)
            r = edge_response(g, v) if A.feature == 'curb' else lane_response(g, v)
        xj, yj, yawj = PRIOR_REL[j]
        c, s = np.cos(yawi), np.sin(yawi)
        dx = c * (xj - xi) + s * (yj - yi)
        dy = -s * (xj - xi) + c * (yj - yi)
        M = pix_affine(dx, dy, yawj - yawi)
        acc += cv2.warpAffine(r, M, (W, H), flags=cv2.INTER_LINEAR, borderValue=0)
        cnt += cv2.warpAffine(v.astype(np.float32), M, (W, H), flags=cv2.INTER_NEAREST, borderValue=0)
    resp = np.where(cnt > 0, acc / np.maximum(cnt, 1), 0).astype(np.float32)
    if A.feature == 'road':  # unknown pixels -> neutral (mean), not "non-road"
        known = cnt > 0
        resp[~known] = resp[known].mean() if known.any() else 0.5
        return resp
    resp[~road_d] = 0
    return resp


def subpix(res, loc):
    x, y = loc
    dx = dy = 0.0
    if 0 < x < res.shape[1] - 1:
        a, b, c = res[y, x - 1], res[y, x], res[y, x + 1]
        den = a - 2 * b + c
        dx = 0.5 * (a - c) / den if den < 0 else 0.0
    if 0 < y < res.shape[0] - 1:
        a, b, c = res[y - 1, x], res[y, x], res[y + 1, x]
        den = a - 2 * b + c
        dy = 0.5 * (a - c) / den if den < 0 else 0.0
    return x + dx, y + dy


def match_frame(i, prior):
    x, y, yaw = prior
    if A.pin_frames:  # map ground corrupted here: assume the road level with the trusted road further on
        import bev as _bev
        lo, hi = [int(v) for v in A.pin_frames.split(':')]
        _bev.GROUND_MODE = 'pinned' if lo <= i < hi else A.ground_mode
        _bev.PIN_Z = A.pin_z
    idx = MAP.query_idx(x, y, 42.0)
    mg, road = map_bev(MAP.pts[idx], MAP.inten[idx], np.array([x, y]), yaw)
    if A.feature == 'road':
        if A.road_map == 'smooth':  # smooth ground connected to the vehicle (low local slope; kerbs cut it)
            ref, kn = map_road_smooth(MAP.pts[idx], np.array([x, y]), yaw)
            ref = ref.copy()
            ref[~kn] = ref[kn].mean() if kn.any() else 0.5  # unknown map cells are neutral
        else:
            ref = (map_road_scan if A.road_map == 'scan' else map_road_region)(MAP.pts[idx], np.array([x, y]), yaw)
        road = np.ones_like(road, bool)
    elif A.feature == 'curb':
        ref = map_curb(MAP.pts[idx], np.array([x, y]), yaw)
        # kerbs sit on the road boundary: compare on the road and up to curb_band metres beyond it
        b = int(A.curb_band / RES)
        road = cv2.dilate(road.astype(np.uint8), np.ones((2 * b + 1, 2 * b + 1), np.uint8)) > 0
        ref[~road] = 0
    elif MARKS is not None:  # global marking map (tools/marking_map.py) instead of the per-frame top-hat
        ref = np.where(road, MARKS.bev(np.array([x, y]), yaw)[0], 0).astype(np.float32)
    else:
        ref = lane_response(mg, road)
        if A.map_min_blob > 0:  # isolated bright map specks (studs, sparse rings) are not markings
            n, lab, st, _ = cv2.connectedComponentsWithStats((ref > 1.0).astype(np.uint8), connectivity=8)
            small = np.flatnonzero(st[:, cv2.CC_STAT_AREA] < A.map_min_blob)
            small = small[small > 0]
            if len(small):
                ref = np.where(np.isin(lab, small), 0, ref).astype(ref.dtype)
    p = int(A.max_shift / RES)
    padded = cv2.copyMakeBorder(ref, p, p, p, p, cv2.BORDER_CONSTANT,
                                value=float(ref.mean()) if A.feature == 'road' else 0).astype(np.float32)
    # road mask from the map is applied in the (shifted) image frame approximately by dilation
    road_d = cv2.dilate(road.astype(np.uint8), np.ones((2 * p + 1, 3), np.uint8)) > 0
    if A.accum > 0:
        base = accumulated_response(i, road_d)
    else:
        if A.feature == 'road':
            base, v = ROADBEV.render(i)
            base = base.copy()
            base[~v] = base[v].mean() if v.any() else 0.5
        elif A.percam and A.feature == 'lane' and A.ground3d:
            base, v = BEV.render_resp_pose(i, *PRIOR_REL[i], lane_response)
        elif A.percam and A.feature == 'lane':
            base, v = (BEV.render_resp_color if A.color else BEV.render_resp)(i, lane_response)
            base = base * road_d
        else:
            g, v = BEV.render_pose(i, *PRIOR_REL[i]) if A.ground3d else BEV.render(i)
            base = edge_response(g, v & road_d) if A.feature == 'curb' else lane_response(g, v & road_d)
    if A.near_only > 0:  # keep only rows with x <= near_only (rows grow towards the vehicle / behind it)
        from bev import X_MAX as _XM
        r0 = int(max(0, (_XM - A.near_only) / RES))
        base = base.copy()
        base[:r0] = base[r0:].mean() if A.feature == 'road' else 0
    def try_yaw(dyaw):
        resp = rotate_raster(base, dyaw)
        if A.range_weight > 0:  # near rows are reliable (IPM resolution / attitude sensitivity grow with range)
            xs = X_MAX - (np.arange(H) + 0.5) * RES
            resp = resp * np.clip(A.range_weight / xs, 0.25, 1.0)[:, None].astype(np.float32)
        if A.feature == 'road':
            resp = np.where(resp == 0, base.mean(), resp).astype(np.float32)
        if resp.max() <= 0:
            return None
        res = cv2.matchTemplate(padded, resp.astype(np.float32), cv2.TM_CCOEFF_NORMED)
        _, mx, _, loc = cv2.minMaxLoc(res)
        return (mx, dyaw, res, loc)

    best = None
    for dyaw in np.radians(np.arange(-A.max_yaw, A.max_yaw + 1e-6, A.yaw_step)):
        r = try_yaw(dyaw)
        if r is not None and (best is None or r[0] > best[0]):
            best = r
    if best is None:
        return None
    if A.yaw_fine > 0:  # fine search around the coarse optimum, then parabolic interpolation of the peak in yaw
        lim = np.radians(A.max_yaw) + 1e-9
        half = 0.6 * A.yaw_step
        grid = [d for d in best[1] + np.radians(np.arange(-half, half + 1e-9, A.yaw_fine)) if abs(d) <= lim]
        fine = [r for r in (try_yaw(d) for d in grid) if r is not None]
        if fine:
            j = int(np.argmax([r[0] for r in fine]))
            best = fine[j]
            if 0 < j < len(fine) - 1:
                a0, a1, a2 = fine[j - 1][0], fine[j][0], fine[j + 1][0]
                den = a0 - 2 * a1 + a2
                if den < 0:
                    off = float(np.clip(0.5 * (a0 - a2) / den, -0.5, 0.5)) * np.radians(A.yaw_fine)
                    best = (best[0], best[1] + off, best[2], best[3])
    mx, dyaw, res, loc = best
    sx, sy = subpix(res, loc)
    # second peak outside a 1 m radius
    r2 = res.copy()
    rad = int(1.0 / RES)
    cv2.circle(r2, loc, rad, -1, -1)
    second = r2.max()
    dc, dr = sx - p, sy - p  # template offset inside padded map raster (pixels)
    # image content at (r, c) matches map at (r + dr, c + dc): map features sit dr rows lower => prior is
    # ahead by dr*RES (x = X_MAX - r*RES) and to the right by dc*RES... sign verified with eval (A.sign)
    fwd = A.sign * dr * RES
    lat = A.sign * dc * RES
    # per-axis distinctiveness: best score >= 1 m away along each axis (narrow band through the peak). Parallel
    # lane lines give a flat ridge along fwd (low sharp_f); crosswalks / stop lines / text make fwd distinct.
    r0, c0 = loc[1], loc[0]
    far = int(1.0 / RES)
    band = res[:, max(c0 - 2, 0):c0 + 3].max(axis=1)
    idx = np.abs(np.arange(len(band)) - r0) >= far
    sharp_f = mx - (band[idx].max() if idx.any() else -1.0)
    band = res[max(r0 - 2, 0):r0 + 3, :].max(axis=0)
    idx = np.abs(np.arange(len(band)) - c0) >= far
    sharp_l = mx - (band[idx].max() if idx.any() else -1.0)
    return mx, mx - second, fwd, lat, A.yaw_sign * dyaw, sharp_f, sharp_l


def worker(chunk):
    out = []
    t0 = time.time()
    for n, i in enumerate(chunk):
        x, y, yaw = PRIOR[i]
        r = match_frame(i, (x, y, yaw))
        if r is None:
            continue
        peak, sharp, fwd, lat, dyaw, sharp_f, sharp_l = r
        c, s = np.cos(yaw), np.sin(yaw)
        meas = (x + c * fwd - s * lat, y + s * fwd + c * lat, yaw + dyaw)
        out.append((i, x, y, yaw, *meas, peak, sharp, fwd, lat, dyaw, sharp_f, sharp_l))
        if n % 50 == 0:
            print(f'[{chunk[0]}] frame {i} {time.time() - t0:.0f}s peak={peak:.3f} sharp={sharp:.3f} '
                  f'fwd={fwd:.2f} lat={lat:.2f} dyaw={np.degrees(dyaw):.2f}', flush=True)
    return out


def init_worker():
    pass  # MAP and BEV are loaded once in the parent and shared copy-on-write after fork


def main():
    global A, PRIOR
    ap = argparse.ArgumentParser()
    ap.add_argument('track_dir')
    ap.add_argument('prior_csv', help='trajectory providing the prior pose per frame (fuse.py output)')
    ap.add_argument('output_npz')
    ap.add_argument('--cam_height', type=float, default=1.5)
    ap.add_argument('--pitch', type=float, default=0.0)
    ap.add_argument('--roll', type=float, default=0.0)
    ap.add_argument('--yaw_mount', type=float, default=-2.0)
    ap.add_argument('--max_shift', type=float, default=2.5)
    ap.add_argument('--max_yaw', type=float, default=3.0)
    ap.add_argument('--yaw_step', type=float, default=0.5)
    ap.add_argument('--yaw_fine', type=float, default=0.0, help='>0: refine the yaw around the coarse optimum in these steps (deg)')
    ap.add_argument('--sign', type=float, default=-1.0)
    ap.add_argument('--yaw_sign', type=float, default=-1.0)
    ap.add_argument('--feature', choices=['lane', 'curb', 'road'], default='lane', help='road markings or kerb edges')
    ap.add_argument('--road_map', choices=['scan', 'fill', 'smooth'], default='smooth', help='map road surface: row scan or flood fill')
    ap.add_argument('--road_v3', action='store_true', help='kerb map v3: road-level criterion vs ramps + hole filling')
    ap.add_argument('--road_seg', default='road', help='segmentation subdir (road = SegFormer-B2, road_m2f = Mask2Former)')
    ap.add_argument('--road_colour', action='store_true', help='average segmentation with a per-frame asphalt colour model')
    ap.add_argument('--curb_band', type=float, default=3.0, help='m beyond the road surface searched for kerbs')
    ap.add_argument('--range_weight', type=float, default=0.0, help='>0: weight rows by clip(x0 / range, 0.25, 1)')
    ap.add_argument('--map_marks', default='', help='marking_map.py npz: map-side lane response from the global marking map')
    ap.add_argument('--near_only', type=float, default=0.0,
                    help='>0: use only the BEV within this many metres ahead (map distortion grows with range)')
    ap.add_argument('--map_min_blob', type=int, default=0, help='>0: drop map marking blobs smaller than this (cells)')
    ap.add_argument('--pin_z', type=float, default=None, help='road height assumed level (map frame) for --pin_frames')
    ap.add_argument('--pin_frames', default='', help='lo:hi frames that use the pinned road height')
    ap.add_argument('--hybrid_ratio', type=float, default=0.6, help='ground_mode hybrid: plane/local road threshold')
    ap.add_argument('--ground_mode', choices=['plane', 'local', 'smooth', 'hybrid', 'profile', 'pinned'], default='plane',
                    help='map road surface reference: one plane under the vehicle, or a smooth local ground surface')
    ap.add_argument('--color', action='store_true', help='with --percam: add yellowness / redness marking channels')
    ap.add_argument('--percam', action='store_true', help='compute the marking response per camera, then blend')
    ap.add_argument('--photonorm', action='store_true', help='per-camera exposure normalisation before blending')
    ap.add_argument('--accum', type=int, default=0, help='+-frames of image responses accumulated')
    ap.add_argument('--accum_step', type=int, default=4)
    ap.add_argument('--accum_dist', type=float, default=0.0, help='>0: accumulate by path distance (+- metres)')
    ap.add_argument('--accum_ds', type=float, default=0.7, help='metres of travel between accumulated frames')
    ap.add_argument('--ground3d', action='store_true', help='IPM onto the map road surface with road tilt')
    ap.add_argument('--dyn_attitude', action='store_true', help='add IMU high-pass pitch/roll (braking, bumps)')
    ap.add_argument('--dyn_roll_only', action='store_true', help='use only the IMU roll (side tilt from bumps)')
    ap.add_argument('--dyn_sign', type=float, default=1.0)
    ap.add_argument('--pitch_per_accel', type=float, default=0.0, help='deg of body pitch per m/s^2 (suspension)')
    ap.add_argument('--frame_list', default='', help='npy of frame indices to process (overrides --frames)')
    ap.add_argument('--prior_noise', type=float, nargs=2, default=[0.0, 0.0])
    ap.add_argument('--frames', default='')
    ap.add_argument('--workers', type=int, default=4)
    A = ap.parse_args()
    global TS_ALL
    TS_ALL, _ = ply_frames(os.path.join(A.track_dir, 'raw_ply'))
    ref = pd.read_csv(A.prior_csv)
    n = len(ref)
    PRIOR = ref[['x', 'y', 'yaw']].values.copy()
    global PRIOR_REL
    PRIOR_REL = PRIOR.copy()  # relative motion for accumulation comes from the un-noised smooth trajectory
    global PATH_S
    PATH_S = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(PRIOR_REL[:, 0]), np.diff(PRIOR_REL[:, 1])))])
    if A.prior_noise[0] > 0 or A.prior_noise[1] > 0:
        rng = np.random.default_rng(0)
        PRIOR[:, :2] += rng.normal(0, A.prior_noise[0], (n, 2))
        PRIOR[:, 2] += np.radians(rng.normal(0, A.prior_noise[1], n))
    lo, hi, st = 0, n, 1
    if A.frames:
        parts = [int(v) for v in A.frames.split(':')]
        lo, hi = parts[:2]
        st = parts[2] if len(parts) > 2 else 1
    frames = list(range(lo, hi, st))
    if A.frame_list:
        frames = [int(v) for v in np.load(A.frame_list)]
    chunks = [frames[k:k + 40] for k in range(0, len(frames), 40)]
    global MAP, BEV, MARKS
    MAP = load_map(A.track_dir)  # loaded once (~1.5 GB), shared by forked workers
    if A.map_marks:
        from marking_map import MarkingMap
        MARKS = MarkingMap(A.map_marks)
    if A.ground3d:  # same dynamic attitude model as the plane IPM below; the static road tilt comes from the map
        ground = GroundRaster(MAP.pts)
        dyn = None
        if A.dyn_attitude:
            from common import Sensors
            from scipy.ndimage import gaussian_filter1d
            S = Sensors(A.track_dir)
            e = S.imu_rot(TS_ALL).as_euler('ZYX', degrees=True)
            hp = lambda v: v - gaussian_filter1d(v, 2.0 * 28, mode='nearest')  # noqa: E731
            dyn = np.stack([A.dyn_sign * hp(e[:, 1]), A.dyn_sign * hp(e[:, 2])], 1)
            if A.dyn_roll_only:
                dyn[:, 0] = 0.0
        if A.pitch_per_accel != 0.0:
            from scipy.ndimage import gaussian_filter1d
            tt = (TS_ALL - TS_ALL[0]) * 1e-9
            v = gaussian_filter1d(np.hypot(np.gradient(PRIOR_REL[:, 0], tt), np.gradient(PRIOR_REL[:, 1], tt)), 14)
            acc = gaussian_filter1d(np.gradient(v, tt), 14)
            dyn = np.zeros((len(acc), 2)) if dyn is None else dyn
            dyn[:, 0] += A.pitch_per_accel * acc
        BEV = ImageBEV3D(A.track_dir, ground, A.cam_height, (A.pitch, A.roll, A.yaw_mount), dyn)
    else:
        BEV = ImageBEV(A.track_dir, A.cam_height, A.pitch, A.roll, A.yaw_mount)
        if A.dyn_attitude:
            from common import Sensors
            from scipy.ndimage import gaussian_filter1d
            S = Sensors(A.track_dir)
            e = S.imu_rot(TS_ALL).as_euler('ZYX', degrees=True)
            hp = lambda v: v - gaussian_filter1d(v, 2.0 * 28, mode='nearest')  # noqa: E731
            BEV.dyn = np.stack([A.dyn_sign * hp(e[:, 1]), A.dyn_sign * hp(e[:, 2])], 1)
            if A.dyn_roll_only:
                BEV.dyn[:, 0] = 0.0
        if A.pitch_per_accel != 0.0:
            # suspension model: body pitch (nose dive when braking, squat when accelerating) = k * longitudinal accel
            from scipy.ndimage import gaussian_filter1d
            tt = (TS_ALL - TS_ALL[0]) * 1e-9
            v = gaussian_filter1d(np.hypot(np.gradient(PRIOR_REL[:, 0], tt), np.gradient(PRIOR_REL[:, 1], tt)), 14)
            acc = gaussian_filter1d(np.gradient(v, tt), 14)
            dyn = np.zeros((len(acc), 2)) if BEV.dyn is None else BEV.dyn
            dyn[:, 0] += A.pitch_per_accel * acc
            BEV.dyn = dyn
    BEV.photonorm = A.photonorm
    import bev as _bevmod
    if _bevmod.CAM_SYNC:  # side cameras capture up to ~13 ms off the frame time: shift their BEV by speed * dt
        tt_ = (TS_ALL - TS_ALL[0]) * 1e-9
        from scipy.ndimage import gaussian_filter1d as _g1
        spd_ = _g1(np.hypot(np.gradient(PRIOR_REL[:, 0], tt_), np.gradient(PRIOR_REL[:, 1], tt_)), 5)
        BEV.set_motion(spd_)
        print('camera capture-time sync on: mean |shift| ' +
              ', '.join(f'{c} {np.mean(np.abs(BEV.tshift[c])):.3f} m' for c in BEV.cams), flush=True)
    import bev as _bev
    _bev.GROUND_MODE = A.ground_mode
    _bev.HYBRID_RATIO = A.hybrid_ratio
    if A.road_v3:
        _bev.ROAD_LEVEL_TOL, _bev.ROAD_FILL_M2 = 0.08, 3.0
    global ROADBEV
    ROADBEV = RoadProbBEV(BEV, A.road_seg, 'seg_colour' if A.road_colour else 'road') if A.feature == 'road' else None
    with mp.get_context('fork').Pool(A.workers, initializer=init_worker) as pool:
        res = pool.map(worker, chunks, chunksize=1)
    rows = np.array([r for c in res for r in c])
    ts, _ = ply_frames(os.path.join(A.track_dir, 'raw_ply'))
    keys = ['idx', 'px', 'py', 'pyaw', 'mx', 'my', 'myaw', 'peak', 'sharp', 'fwd', 'lat', 'dyaw', 'sharp_f', 'sharp_l']
    outd = {k: rows[:, j] for j, k in enumerate(keys)}
    outd['idx'] = outd['idx'].astype(int)
    outd['ts'] = ts[outd['idx']]
    outd['max_yaw'] = np.full(len(rows), A.max_yaw)  # per-fix yaw search limit (fuse drops fixes at the limit)
    np.savez(A.output_npz, **outd)
    print('saved', A.output_npz, len(rows))


if __name__ == '__main__':
    main()
