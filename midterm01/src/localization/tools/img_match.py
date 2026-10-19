#!/usr/bin/env python3
"""Map-to-image localization without a BEV: 3D map markings projected straight into the raw camera images.

Per fix frame i and pose hypothesis (dx fwd, dy left, dyaw) in i's vehicle frame, the marking-map points (mask mode,
true map heights) within range are moved by the inverse correction, taken into each neighbour frame j's vehicle
frame (prior relative motion, road tilt under the vehicle from the map ground) and projected into the three
cameras. Score = mean image marking response at the projected points (white top-hat / local std, times the
segmentation road probability, blurred), over all cameras and neighbour frames. No flat-ground IPM, no resampling:
far markings keep their geometry, each camera its own calibration. Coarse grid + fine grid + parabolic peak.
Output npz has the bev_match fields (idx, ts, px, py, pyaw, mx, my, myaw, peak, sharp, fwd, lat, dyaw, sharp_f,
sharp_l, max_yaw) so fuse.py --bev and the evaluation tools take it unchanged.
"""
import argparse
import os
import sys
import time
from multiprocessing import Pool

import cv2
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
from bev import ImageBEV  # noqa: E402
from common import ply_frames  # noqa: E402
from marking_map import MarkingMap, road_level, vehicle_frame  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('prior_csv')
ap.add_argument('marks_npz')
ap.add_argument('out_npz')
ap.add_argument('--frames', default='450:-1:15')
ap.add_argument('--neigh', default='-6,-3,0,3,6', help='neighbour frame offsets accumulated per fix')
ap.add_argument('--range', type=float, default=35.0)
ap.add_argument('--npts', type=int, default=1500)
ap.add_argument('--xy', type=float, default=1.0, help='search half width (m)')
ap.add_argument('--yaw', type=float, default=1.0, help='search half width (deg)')
ap.add_argument('--workers', type=int, default=4)
ap.add_argument('--yaw_mount', type=float, default=-1.5)
ap.add_argument('--cams', default='f,fl,fr')
ap.add_argument('--blur_coarse', type=float, default=4.0)
ap.add_argument('--blur_fine', type=float, default=1.5)
a = ap.parse_args()

tr = pd.read_csv(a.prior_csv)
ts, _ = ply_frames(os.path.join(a.track_dir, 'raw_ply'))
MK = MarkingMap(a.marks_npz)
B = ImageBEV(a.track_dir, 1.5, 0, 0, a.yaw_mount)
C = B.C
CAMS = a.cams.split(',')
Rm = Rotation.from_euler('ZYX', [a.yaw_mount, 0, 0], degrees=True).as_matrix()
RD = {c: np.load(os.path.join(a.track_dir, 'road', f'{c}.npz'))['road'] for c in CAMS}
NEIGH = [int(v) for v in a.neigh.split(',')]
SYNC = None
if os.environ.get('SDC_CAM_SYNC', '0') == '1':   # side cameras capture up to ~13 ms off the frame time
    from scipy.ndimage import gaussian_filter1d
    from bev import camera_time_offsets
    _tt = (ts - ts[0]) * 1e-9
    _spd = gaussian_filter1d(np.hypot(np.gradient(tr.x.values, _tt[:len(tr)]), np.gradient(tr.y.values, _tt[:len(tr)])), 5)
    _dts = camera_time_offsets(a.track_dir, tuple(CAMS))
    SYNC = {c: _spd * np.asarray(_dts[c])[:len(_spd)] for c in CAMS}
rng = np.random.default_rng(0)
A_COARSE, A_FINE = a.blur_coarse, a.blur_fine


def response(j, c):
    img = cv2.imread(os.path.join(B.track_dir, f'{B.track}_images', f'{B.track}_{c}_images', B.names[c][j]), cv2.IMREAD_GRAYSCALE)
    # paint: clearly brighter than the local road surface (median), not texture amplified by a local std
    small = cv2.resize(img, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
    med = cv2.resize(cv2.medianBlur(small, 15), (img.shape[1], img.shape[0]), interpolation=cv2.INTER_LINEAR).astype(np.float32)
    r = np.clip((img.astype(np.float32) - med - 15.0) / 30.0, 0, 1)
    rp = cv2.resize(RD[c][j].astype(np.float32) / 255.0, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_LINEAR)
    r *= np.clip(rp * 1.5, 0, 1)                      # markings on the road only
    bad = B.ego_bad(c, img.shape).copy()
    if c in B.masks:
        m = np.unpackbits(B.masks[c][j], axis=1)[:, :img.shape[1] // 2]
        bad |= cv2.resize(m, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
    r[cv2.dilate(bad, np.ones((9, 9), np.uint8)) > 0] = 0
    return cv2.GaussianBlur(r, (0, 0), A_COARSE), cv2.GaussianBlur(r, (0, 0), A_FINE)


def match(i):
    x, y, yaw = tr.x[i], tr.y[i], tr.yaw[i]
    Pw = MK.points(x, y, a.range + 8)
    if len(Pw) < 100:
        return None
    q0 = vehicle_frame(Pw, (x, y, yaw), MK.height)
    keep = road_level(q0) & (q0[:, 0] > -8) & (np.hypot(q0[:, 0], q0[:, 1]) < a.range)
    Pw = Pw[keep]
    if len(Pw) < 100:
        return None
    # keep markings that project onto the road (image segmentation at the prior pose): verges / sidewalks out
    onroad = np.zeros(len(Pw), bool)
    Q0 = vehicle_frame(Pw, (x, y, yaw), MK.height) @ Rm
    for c in CAMS:
        uv, ok = C.project(c, Q0)
        if ok.any():
            h4, w4 = RD[c].shape[1:]
            u4 = np.clip((uv[ok, 0] / 4).astype(int), 0, w4 - 1)
            v4 = np.clip((uv[ok, 1] / 4).astype(int), 0, h4 - 1)
            onroad[np.flatnonzero(ok)[RD[c][i][v4, u4] > 128]] = True
    Pw = Pw[onroad]
    if len(Pw) < 50:
        return None
    if len(Pw) > a.npts:
        Pw = Pw[rng.choice(len(Pw), a.npts, replace=False)]
    Bw = MK.background(x, y, a.range + 8, a.npts, rng)
    qb = vehicle_frame(Bw, (x, y, yaw), MK.height)
    Bw = Bw[road_level(qb, tol=0.15, slope=0.02) & (qb[:, 0] > -8) & (np.hypot(qb[:, 0], qb[:, 1]) < a.range)]
    onb = np.zeros(len(Bw), bool)
    Qb = vehicle_frame(Bw, (x, y, yaw), MK.height) @ Rm
    for c in CAMS:
        uv, ok = C.project(c, Qb)
        if ok.any():
            h4, w4 = RD[c].shape[1:]
            onb[np.flatnonzero(ok)[RD[c][i][np.clip((uv[ok, 1] / 4).astype(int), 0, h4 - 1), np.clip((uv[ok, 0] / 4).astype(int), 0, w4 - 1)] > 128]] = True
    Bw = Bw[onb][:a.npts]
    nM = len(Pw)
    Pw = np.concatenate([Pw, Bw])
    frames = []
    for o in NEIGH:
        j = i + o
        if j < 0 or j >= len(tr):
            continue
        xj, yj, yj_ = tr.x[j], tr.y[j], tr.yaw[j]
        # vehicle pose j: camera position / rotation with the road tilt (as vehicle_frame), computed once
        zero = vehicle_frame(np.array([[xj, yj, 0.0]]), (xj, yj, yj_), MK.height)
        e = vehicle_frame(np.eye(3) + np.array([xj, yj, 0.0]), (xj, yj, yj_), MK.height) - zero
        frames.append((np.array([xj, yj, 0.0]), zero[0], e, {c: response(j, c) for c in CAMS}, j))
    c_, s_ = np.cos(yaw), np.sin(yaw)

    def score(dx, dy, dyaw, lev=0):
        # pose correction in i's frame <=> inverse correction applied to the map points (about the vehicle at i)
        cy, sy = np.cos(-dyaw), np.sin(-dyaw)
        tx, ty = c_ * dx - s_ * dy, s_ * dx + c_ * dy
        px, py = Pw[:, 0] - x - tx, Pw[:, 1] - y - ty
        P = np.stack([x + cy * px - sy * py, y + sy * px + cy * py, Pw[:, 2]], 1)
        sm = sb = 0.0
        nm = nb = 0
        isM = np.arange(len(P)) < nM
        for (org, zero, e, resp, j) in frames:
            q = (P - org) @ e + zero                   # affine world -> vehicle frame of j
            for c in CAMS:
                if SYNC is not None and SYNC[c][j] != 0:
                    qc = q.copy()
                    qc[:, 0] -= SYNC[c][j]             # captured later -> vehicle further along -> scene nearer
                    Q = qc @ Rm
                else:
                    Q = q @ Rm
                uv, ok = C.project(c, Q)
                if ok.sum() < 20:
                    continue
                v = cv2.remap(resp[c][lev], uv[ok, 0].astype(np.float32).reshape(-1, 1), uv[ok, 1].astype(np.float32).reshape(-1, 1), cv2.INTER_LINEAR).ravel()
                mk = isM[ok]
                sm += float(v[mk].sum()); nm += int(mk.sum())
                sb += float(v[~mk].sum()); nb += int((~mk).sum())
        # markings minus road background: alignment, not the brightness of the region the points fall into
        return sm / max(nm, 1) - sb / max(nb, 1)

    best = (-1, 0, 0, 0)
    gx = np.arange(-a.xy, a.xy + 1e-9, 0.2)
    gyaw = np.radians(np.arange(-a.yaw, a.yaw + 1e-9, 0.25))
    S = np.zeros((len(gx), len(gx), len(gyaw)))
    for p, dx in enumerate(gx):
        for q, dy in enumerate(gx):
            for r_, dw in enumerate(gyaw):
                S[p, q, r_] = score(dx, dy, dw)
    p, q, r_ = np.unravel_index(np.argmax(S), S.shape)
    best = (S[p, q, r_], gx[p], gx[q], gyaw[r_])
    # fine grid around the coarse optimum, then parabolic refinement per axis
    fx = best[1] + np.arange(-0.15, 0.151, 0.05)
    fy = best[2] + np.arange(-0.15, 0.151, 0.05)
    fw = best[3] + np.radians(np.arange(-0.2, 0.201, 0.05))
    F = np.array([[[score(u, v, w, 1) for w in fw] for v in fy] for u in fx])
    p2, q2, r2 = np.unravel_index(np.argmax(F), F.shape)

    def para(a0, a1, a2):
        den = a0 - 2 * a1 + a2
        return float(np.clip(0.5 * (a0 - a2) / den, -0.5, 0.5)) if den < 0 else 0.0
    dx = fx[p2] + (para(F[p2 - 1, q2, r2], F[p2, q2, r2], F[p2 + 1, q2, r2]) * 0.05 if 0 < p2 < len(fx) - 1 else 0)
    dy = fy[q2] + (para(F[p2, q2 - 1, r2], F[p2, q2, r2], F[p2, q2 + 1, r2]) * 0.05 if 0 < q2 < len(fy) - 1 else 0)
    dw = fw[r2] + (para(F[p2, q2, r2 - 1], F[p2, q2, r2], F[p2, q2, r2 + 1]) * np.radians(0.05) if 0 < r2 < len(fw) - 1 else 0)
    peak = float(F[p2, q2, r2])
    # distinctiveness per axis (coarse grid): peak minus best score >= 0.6 m away along that axis
    rowf = S[:, q, :].max(axis=1)
    far = np.abs(gx - gx[p]) >= 0.6
    sharp_f = peak - (rowf[far].max() if far.any() else 0)
    rowl = S[p, :, :].max(axis=1)
    far = np.abs(gx - gx[q]) >= 0.6
    sharp_l = peak - (rowl[far].max() if far.any() else 0)
    second = np.sort(S.ravel())[-2]
    mx, my = x + c_ * dx - s_ * dy, y + s_ * dx + c_ * dy
    return (i, ts[i], x, y, yaw, mx, my, yaw + dw, peak, peak - second, dx, dy, dw, sharp_f, sharp_l, a.yaw)


def main():
    lo, hi, st = (int(v) for v in a.frames.split(':'))
    frames = list(range(lo, len(tr) if hi < 0 else hi, st))
    t0 = time.time()
    out = []
    with Pool(a.workers) as pool:
        for n, r in enumerate(pool.imap(match, frames, chunksize=2)):
            if r is not None:
                out.append(r)
            if n % 20 == 0:
                print(f'{n}/{len(frames)} {time.time() - t0:.0f}s', flush=True)
    keys = ['idx', 'ts', 'px', 'py', 'pyaw', 'mx', 'my', 'myaw', 'peak', 'sharp', 'fwd', 'lat', 'dyaw', 'sharp_f', 'sharp_l', 'max_yaw']
    r = np.array(out, dtype=np.float64)
    d = {k: r[:, j] for j, k in enumerate(keys)}
    d['idx'] = d['idx'].astype(int)
    d['ts'] = np.array([o[1] for o in out], dtype=np.int64)
    np.savez(a.out_npz, **d)
    print(f'saved {a.out_npz} {len(out)} fixes, {time.time() - t0:.0f}s')


if __name__ == '__main__':
    main()
