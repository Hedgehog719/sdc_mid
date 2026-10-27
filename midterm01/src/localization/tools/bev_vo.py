#!/usr/bin/env python3
"""Visual odometry from the road surface (GNSS-free speed, Track03).

Without GNSS or wheel speed the along-track motion has to come from the images: the front camera image is mapped
onto the road plane (inverse perspective with the calibrated fisheye model, 2 cm cells, 5.5..12 m ahead, +-2.5 m
lateral; vehicle frame with the road 1.5 m below the camera reference) and consecutive ground images are
phase-correlated: the road texture / markings shift by the vehicle's displacement (metric, no point-cloud scale).
The band-pass is applied in the camera image (masking after filtering creates static edges that lock a whitened
correlation at zero shift); a fully valid central block of frame j is template-matched in frame j + step, and the
peak is tracked near the previous pair's shift (the speed cannot jump within 30 ms).
Output per frame pair (j, j + step): forward / lateral displacement (m), peak strength; speed = forward / dt.
usage: bev_vo.py <track_dir> <out.npz> [--step 1 --workers 4]
"""
import argparse
import os
import sys
from multiprocessing import Pool

import cv2
cv2.setNumThreads(1)
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
from bev import ImageBEV  # noqa: E402
from common import ply_frames  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('out')
ap.add_argument('--step', type=int, default=1)
ap.add_argument('--x0', type=float, default=5.5)
ap.add_argument('--x1', type=float, default=12.0)
ap.add_argument('--y1', type=float, default=2.5)
ap.add_argument('--res', type=float, default=0.02)
ap.add_argument('--max_fwd', type=float, default=0.8, help='m per pair kept in the correlation crop')
ap.add_argument('--win', type=int, default=12, help='px: peak search window around the predicted shift')
ap.add_argument('--no_imu', action='store_true')
ap.add_argument('--yaw_mount', type=float, default=-1.5)
ap.add_argument('--workers', type=int, default=4)
ap.add_argument('--mask_tol', type=float, default=0.9, help='minimum valid fraction of the template block; between this '
                'and 0.9 the block is matched with its validity mask (a lead vehicle close ahead hides the far part of '
                'the road band, the road just ahead of the hood stays visible); 0.9 = full blocks only')
a = ap.parse_args()

T = os.path.basename(os.path.normpath(a.track_dir))
TS, _ = ply_frames(os.path.join(a.track_dir, 'raw_ply'))
TS = np.asarray(TS, np.int64)
B = ImageBEV(a.track_dir, 1.5, 0, 0, a.yaw_mount)
C = B.C
Rm = Rotation.from_euler('ZYX', [a.yaw_mount, 0, 0], degrees=True).as_matrix()
xs = np.arange(a.x1, a.x0, -a.res)            # rows: far -> near (x forward)
ys = np.arange(a.y1, -a.y1, -a.res)           # cols: left -> right
GX, GY = np.meshgrid(xs, ys, indexing='ij')
PTS = np.stack([GX.ravel(), GY.ravel(), np.full(GX.size, -1.5)], 1)   # road 1.5 m below the camera reference
UV, OK = C.project('f', PTS @ Rm)
MAPX = UV[:, 0].reshape(GX.shape).astype(np.float32)
MAPY = UV[:, 1].reshape(GX.shape).astype(np.float32)
VALID = OK.reshape(GX.shape)
HANN = cv2.createHanningWindow((GX.shape[1], GX.shape[0]), cv2.CV_32F)
CR = int(a.max_fwd / a.res)                    # crop: rows -CR..+CR, cols -CC..+CC (shift of g1 vs g0)
CC = 12
STATIC = None


def ground(j):
    """Band-passed road image on the ground plane (band-pass in the camera image: no artificial mask edges)."""
    img = cv2.imread(os.path.join(a.track_dir, f'{T}_images', f'{T}_f_images', B.names['f'][j]), cv2.IMREAD_GRAYSCALE)
    img = img.astype(np.float32)
    bad = B.ego_bad('f', img.shape).copy()
    if 'f' in B.masks and j < len(B.masks['f']):
        m = np.unpackbits(B.masks['f'][j], axis=1)[:, :img.shape[1] // 2]
        bad |= cv2.resize(m, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
    bad = cv2.dilate(bad, np.ones((15, 15), np.uint8)) > 0
    hp = cv2.GaussianBlur(img, (0, 0), 1.0) - cv2.GaussianBlur(img, (0, 0), 6.0)
    sd = np.sqrt(np.maximum(cv2.GaussianBlur(hp * hp, (0, 0), 15), 1.0))
    hp = np.where(bad, 0, hp / sd).astype(np.float32)
    return cv2.remap(hp, MAPX, MAPY, cv2.INTER_LINEAR, borderValue=0), \
        cv2.remap((~bad).astype(np.float32), MAPX, MAPY, cv2.INTER_NEAREST, borderValue=0) > 0.5


def surface(j):
    """Template match: the central block of ground(j) searched in ground(j + step); entry [CR + dr, CC + dc] = shift
    of the content (rows + = towards the vehicle = forward motion). Blocks touching a masked area are neutral."""
    g0, v0 = ground(j)
    g1, v1 = ground(j + a.step)
    rA, rB, cA, cB = CR, g0.shape[0] - CR, CC + 20, g0.shape[1] - CC - 20
    tpl = g0[rA:rB, cA:cB]
    vt = v0[rA:rB, cA:cB]
    frac = vt.mean()
    if frac < min(a.mask_tol, 0.9) or (tpl.std() if frac >= 0.9 else tpl[vt].std()) < 1e-3:   # (full blocks: as before)
        return j, None
    if frac >= 0.9:
        res = cv2.matchTemplate(g1, tpl, cv2.TM_CCOEFF_NORMED)      # (2CR + 1, 2CC + 41) over shifts
    else:   # partly hidden block: masked normalised correlation over its visible road only
        res = cv2.matchTemplate(g1, tpl, cv2.TM_CCOEFF_NORMED, mask=vt.astype(np.float32))
        res = np.nan_to_num(res, nan=0.0, posinf=0.0, neginf=0.0)
    return j, res[:, 20:20 + 2 * CC + 1].astype(np.float32)


if __name__ == '__main__':
    n = len(TS)
    with Pool(a.workers) as p:
        S = dict(p.map(surface, range(0, n - a.step), chunksize=16))
    # predicted shift per pair: IMU along-track velocity (integrated from rest; only guides the search window, the
    # measured shift comes from the images), else the previous pair's shift
    vpred = None
    fimu = os.path.join(a.track_dir, 'sensors', 'imu.csv')
    if os.path.exists(fimu) and not a.no_imu:
        import pandas as pd
        im = pd.read_csv(fimu)
        q_ = Rotation.from_quat(im[['qx', 'qy', 'qz', 'qw']].values)
        aw = q_.apply(im[['ax', 'ay', 'az']].values)
        yw_ = np.unwrap(q_.as_euler('ZYX')[:, 0])
        along = np.cos(yw_) * aw[:, 0] + np.sin(yw_) * aw[:, 1]
        ti = im.t.values.astype(np.float64)
        ca = np.concatenate([[0], np.cumsum(0.5 * (along[1:] + along[:-1]) * np.diff(ti) * 1e-9)])
        vpred = np.interp(TS.astype(np.float64), ti, ca) - np.interp(float(TS[0]), ti, ca)
    # sequential peak tracking (rows: + = content moved towards the vehicle = forward motion)
    pred = None
    out = []
    vo_v = 0.0          # running VO speed (m/s) used to re-anchor the IMU prediction (removes its drift)
    imu_off = 0.0
    for j in range(0, n - a.step):
        if vpred is not None and pred is not None:
            dtj = (TS[j + a.step] - TS[j]) * 1e-9
            pred = ((vpred[j] + imu_off) * dtj / a.res, pred[1])
        if S[j] is None:   # masked (lead vehicle close ahead): keep the prediction
            dr, dc = pred if pred is not None else (0.0, 0.0)
            out.append((j, j + a.step, dr * a.res, dc * a.res, 0.0))
            continue
        sm = cv2.GaussianBlur(S[j], (0, 0), 0.8)
        if pred is None:          # first pair: global peak
            r, c = np.unravel_index(np.argmax(sm), sm.shape)
        else:
            r0 = int(round(CR + pred[0]))
            c0_ = int(round(CC + pred[1]))
            lo_r, hi_r = max(r0 - a.win, 1), min(r0 + a.win + 1, sm.shape[0] - 1)
            lo_c, hi_c = max(c0_ - 4, 1), min(c0_ + 5, sm.shape[1] - 1)
            sub = sm[lo_r:hi_r, lo_c:hi_c]
            rr, cc = np.unravel_index(np.argmax(sub), sub.shape)
            r, c = lo_r + rr, lo_c + cc
        r, c = int(np.clip(r, 1, sm.shape[0] - 2)), int(np.clip(c, 1, sm.shape[1] - 2))

        def para(v):
            den = v[0] - 2 * v[1] + v[2]
            return float(np.clip(0.5 * (v[0] - v[2]) / den, -0.5, 0.5)) if den < 0 else 0.0
        dr = r - CR + para(sm[r - 1:r + 2, c])
        dc = c - CC + para(sm[r, c - 1:c + 2])
        strength = float(sm[r, c])
        if strength < 0.3 and pred is not None:   # weak peak: keep the prediction
            dr, dc = pred
        elif vpred is not None:   # re-anchor the IMU velocity to the measured motion (slow: 2 s time constant)
            dtj = (TS[j + a.step] - TS[j]) * 1e-9
            v_meas = dr * a.res / max(dtj, 1e-3)
            imu_off += (v_meas - (vpred[j] + imu_off)) * min(dtj / 2.0, 1.0)
        pred = (dr, dc)
        out.append((j, j + a.step, dr * a.res, dc * a.res, strength))
    R = np.array(out)
    dt = (TS[R[:, 1].astype(int)] - TS[R[:, 0].astype(int)]) * 1e-9
    np.savez(a.out, j=R[:, 0].astype(int), k=R[:, 1].astype(int), fwd=R[:, 2], lat=R[:, 3], resp=R[:, 4], dt=dt,
             speed=R[:, 2] / dt)
    sp = R[:, 2] / dt
    print(f'{T}: {len(R)} pairs, speed median {np.median(sp):.2f} m/s (p10 {np.percentile(sp, 10):.2f}, p90 '
          f'{np.percentile(sp, 90):.2f}), peak strength median {np.median(R[:, 4]):.1f} -> {a.out}')
