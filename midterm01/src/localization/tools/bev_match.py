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
from bev import H, RES, W, X_MAX, Y_MAX, GroundRaster, ImageBEV, ImageBEV3D, lane_response, map_bev  # noqa: E402
from common import TiledMap, ply_frames, wrap  # noqa: E402

A = None


def load_map(track_dir):
    M = TiledMap(os.path.join(track_dir, 'map.npy'), keep_order=True)
    M.inten = np.load(os.path.join(track_dir, 'map_intensity.npy'), mmap_mode='r')[M.order]
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


def accumulated_response(i, road_d):
    """Image lane response of frame i, averaged with neighbours warped by the prior's relative motion."""
    xi, yi, yawi = PRIOR_REL[i]
    acc = np.zeros((H, W), np.float32)
    cnt = np.zeros((H, W), np.float32)
    for j in range(i - A.accum, i + A.accum + 1, A.accum_step):
        if j < 0 or j >= len(PRIOR_REL):
            continue
        g, v = BEV.render_pose(j, *PRIOR_REL[j]) if A.ground3d else BEV.render(j)
        r = lane_response(g, v)
        xj, yj, yawj = PRIOR_REL[j]
        c, s = np.cos(yawi), np.sin(yawi)
        dx = c * (xj - xi) + s * (yj - yi)
        dy = -s * (xj - xi) + c * (yj - yi)
        M = pix_affine(dx, dy, yawj - yawi)
        acc += cv2.warpAffine(r, M, (W, H), flags=cv2.INTER_LINEAR, borderValue=0)
        cnt += cv2.warpAffine(v.astype(np.float32), M, (W, H), flags=cv2.INTER_NEAREST, borderValue=0)
    resp = np.where(cnt > 0, acc / np.maximum(cnt, 1), 0).astype(np.float32)
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
    idx = MAP.query_idx(x, y, 42.0)
    mg, road = map_bev(MAP.pts[idx], MAP.inten[idx], np.array([x, y]), yaw)
    ref = lane_response(mg, road)
    p = int(A.max_shift / RES)
    padded = cv2.copyMakeBorder(ref, p, p, p, p, cv2.BORDER_CONSTANT, value=0).astype(np.float32)
    # road mask from the map is applied in the (shifted) image frame approximately by dilation
    road_d = cv2.dilate(road.astype(np.uint8), np.ones((2 * p + 1, 3), np.uint8)) > 0
    if A.accum > 0:
        base = accumulated_response(i, road_d)
    else:
        g, v = BEV.render_pose(i, *PRIOR_REL[i]) if A.ground3d else BEV.render(i)
        base = lane_response(g, v & road_d)
    best = None
    for dyaw in np.radians(np.arange(-A.max_yaw, A.max_yaw + 1e-6, A.yaw_step)):
        resp = rotate_raster(base, dyaw)
        if resp.max() <= 0:
            continue
        res = cv2.matchTemplate(padded, resp.astype(np.float32), cv2.TM_CCOEFF_NORMED)
        _, mx, _, loc = cv2.minMaxLoc(res)
        if best is None or mx > best[0]:
            best = (mx, dyaw, res, loc)
    if best is None:
        return None
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
    return mx, mx - second, fwd, lat, A.yaw_sign * dyaw


def worker(chunk):
    out = []
    t0 = time.time()
    for n, i in enumerate(chunk):
        x, y, yaw = PRIOR[i]
        r = match_frame(i, (x, y, yaw))
        if r is None:
            continue
        peak, sharp, fwd, lat, dyaw = r
        c, s = np.cos(yaw), np.sin(yaw)
        meas = (x + c * fwd - s * lat, y + s * fwd + c * lat, yaw + dyaw)
        out.append((i, x, y, yaw, *meas, peak, sharp, fwd, lat, dyaw))
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
    ap.add_argument('--sign', type=float, default=-1.0)
    ap.add_argument('--yaw_sign', type=float, default=-1.0)
    ap.add_argument('--accum', type=int, default=0, help='+-frames of image responses accumulated')
    ap.add_argument('--accum_step', type=int, default=4)
    ap.add_argument('--ground3d', action='store_true', help='IPM onto the map road surface with road tilt')
    ap.add_argument('--dyn_attitude', action='store_true', help='add IMU high-pass pitch/roll (braking, bumps)')
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
    global MAP, BEV
    MAP = load_map(A.track_dir)  # loaded once (~1.5 GB), shared by forked workers
    if A.ground3d:
        ground = GroundRaster(MAP.pts)
        dyn = None
        if A.dyn_attitude:
            from common import Sensors
            from scipy.ndimage import gaussian_filter1d
            S = Sensors(A.track_dir)
            e = S.imu_rot(TS_ALL).as_euler('ZYX', degrees=True)
            hp = lambda v: v - gaussian_filter1d(v, 2.0 * 28, mode='nearest')  # noqa: E731
            dyn = np.stack([A.dyn_sign * hp(e[:, 1]), A.dyn_sign * hp(e[:, 2])], 1)
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
        if A.pitch_per_accel != 0.0:
            # suspension model: body pitch (nose dive when braking, squat when accelerating) = k * longitudinal accel
            from scipy.ndimage import gaussian_filter1d
            tt = (TS_ALL - TS_ALL[0]) * 1e-9
            v = gaussian_filter1d(np.hypot(np.gradient(PRIOR_REL[:, 0], tt), np.gradient(PRIOR_REL[:, 1], tt)), 14)
            acc = gaussian_filter1d(np.gradient(v, tt), 14)
            dyn = np.zeros((len(acc), 2)) if BEV.dyn is None else BEV.dyn
            dyn[:, 0] += A.pitch_per_accel * acc
            BEV.dyn = dyn
    with mp.get_context('fork').Pool(A.workers, initializer=init_worker) as pool:
        res = pool.map(worker, chunks, chunksize=1)
    rows = np.array([r for c in res for r in c])
    ts, _ = ply_frames(os.path.join(A.track_dir, 'raw_ply'))
    keys = ['idx', 'px', 'py', 'pyaw', 'mx', 'my', 'myaw', 'peak', 'sharp', 'fwd', 'lat', 'dyaw']
    outd = {k: rows[:, j] for j, k in enumerate(keys)}
    outd['idx'] = outd['idx'].astype(int)
    outd['ts'] = ts[outd['idx']]
    np.savez(A.output_npz, **outd)
    print('saved', A.output_npz, len(rows))


if __name__ == '__main__':
    main()
