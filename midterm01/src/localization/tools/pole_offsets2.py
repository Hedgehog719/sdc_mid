#!/usr/bin/env python3
"""Pole image offsets with a ridge detector (dark or bright poles, cluttered backgrounds), per height band.

For every frame / camera / curated map pole 4-35 m away, the pole (1.5 m .. min(top, 8 m) above the ground) is
projected with the current calibration (optionally capture-time synced, SDC_CAM_SYNC=1). Along the projected curve
the image rows are scanned +-W px; a second-derivative-of-Gaussian filter (scale from the expected pole width) gives
a ridge response whose aligned average peaks at the pole. The offset is measured for the whole pole and for its
lower / middle / upper third (with their mean image rows) so that row-dependent effects (rolling shutter) can be
separated from constant ones (rotation) and speed-dependent ones (timing).
"""
import argparse
import os
import sys

import cv2
import numpy as np
import pandas as pd
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
from camera_model import Cameras  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('traj_csv')
ap.add_argument('out_npz')
ap.add_argument('--step', type=int, default=3)
ap.add_argument('--win', type=int, default=40)
ap.add_argument('--diam', type=float, default=0.25, help='assumed pole diameter (m) for the filter scale')
a = ap.parse_args()
track = os.path.basename(os.path.normpath(a.track_dir))
tr = pd.read_csv(a.traj_csv)
poles = np.load(os.path.join(a.track_dir, 'poles_curated.npy'))
C = Cameras(os.path.join(a.track_dir, f'{track}_images', 'camera_info'))
Rm = Rotation.from_euler('ZYX', [-1.5, 0, 0], degrees=True).as_matrix()
cams = ['fl', 'f', 'fr']
imgdir = {c: os.path.join(a.track_dir, f'{track}_images', f'{track}_{c}_images') for c in cams}
names = {c: sorted(os.listdir(imgdir[c])) for c in cams}
sys.path.insert(0, os.path.dirname(__file__))
from common import ply_frames  # noqa: E402
ts, _ = ply_frames(os.path.join(a.track_dir, 'raw_ply'))
tt = (ts - ts[0]) * 1e-9
spd = gaussian_filter1d(np.hypot(np.gradient(tr.x.values, tt), np.gradient(tr.y.values, tt)), 5)
TSH = None
if os.environ.get('SDC_CAM_SYNC', '0') == '1':
    from bev import camera_time_offsets
    dts = camera_time_offsets(a.track_dir)
    TSH = {c: spd * dts[c] for c in cams}
W = a.win
off_axis = np.arange(-W, W + 1)


def ridge(profiles, sig):
    """Aligned mean of the 2nd-derivative-of-Gaussian response of row profiles -> (offset, score, sign, distinct)."""
    P = gaussian_filter1d(np.asarray(profiles, np.float32), sig, axis=1, order=2, mode='nearest')
    m = P.mean(0)
    edge = int(3 * sig) + 1
    mm = np.abs(m.copy())
    mm[:edge] = 0
    mm[-edge:] = 0
    j = int(np.argmax(mm))
    if mm[j] <= 0 or j <= edge or j >= len(m) - edge - 1:
        return np.nan, 0.0, 0.0, 0.0
    a0, a1, a2 = mm[j - 1], mm[j], mm[j + 1]
    den = a0 - 2 * a1 + a2
    sub = 0.5 * (a0 - a2) / den if den < 0 else 0.0
    excl = max(4, int(2.5 * sig))
    rest = mm.copy()
    rest[max(0, j - excl):j + excl + 1] = 0
    distinct = a1 / (rest.max() + 1e-6)
    return off_axis[j] + sub, a1 / (np.std(m) + 1e-6), float(np.sign(m[j])), distinct


def band_offset(profiles, sig, sign, center):
    """Offset of the same-polarity ridge near `center` (+-8 px) for a subset of rows."""
    if len(profiles) < 3:
        return np.nan
    P = gaussian_filter1d(np.asarray(profiles, np.float32), sig, axis=1, order=2, mode='nearest')
    m = P.mean(0) * sign
    c0 = int(round(center)) + W
    lo, hi = max(1, c0 - 8), min(len(m) - 2, c0 + 8)
    if hi <= lo:
        return np.nan
    j = lo + int(np.argmax(m[lo:hi + 1]))
    a0, a1, a2 = m[j - 1], m[j], m[j + 1]
    den = a0 - 2 * a1 + a2
    sub = 0.5 * (a0 - a2) / den if den < 0 else 0.0
    return off_axis[j] + sub


rows = []
for i in range(0, len(tr), a.step):
    x, y, yaw = tr.x[i], tr.y[i], tr.yaw[i]
    c_, s_ = np.cos(yaw), np.sin(yaw)
    d = poles[:, :2] - [x, y]
    pu, pv = c_ * d[:, 0] + s_ * d[:, 1], -s_ * d[:, 0] + c_ * d[:, 1]
    rng = np.hypot(pu, pv)
    near = np.flatnonzero((rng > 4) & (rng < 35) & (pu > -8))
    if not len(near):
        continue
    for cam in cams:
        img = None
        K = C.cams[cam][0]
        for k in near:
            hs = np.linspace(1.5, min(poles[k, 4], 8.0), 36)
            if hs[-1] - hs[0] < 2.0:
                continue
            sh = TSH[cam][i] if TSH is not None else 0.0
            Q = np.stack([np.full(len(hs), pu[k] - sh), np.full(len(hs), pv[k]), hs - 1.5], 1) @ Rm
            uv, ok = C.project(cam, Q)
            if ok.sum() < 12 or np.ptp(uv[ok, 1]) < 50:
                continue
            if img is None:
                img = cv2.imread(os.path.join(imgdir[cam], names[cam][i]), cv2.IMREAD_GRAYSCALE).astype(np.float32)
            prof, vrow, hgt = [], [], []
            for (u, v), hh in zip(uv[ok], hs[ok]):
                ui, vi = int(round(u)), int(round(v))
                if ui - W < 0 or ui + W + 1 > img.shape[1] or vi < 0 or vi >= img.shape[0]:
                    continue
                r = img[vi, ui - W:ui + W + 1]
                prof.append(r - np.median(r))
                vrow.append(v)
                hgt.append(hh)
            if len(prof) < 12:
                continue
            wpx = K[0, 0] * a.diam / max(rng[k], 1.0)
            sig = float(np.clip(wpx / 2.5, 1.0, 6.0))
            off, score, sign, distinct = ridge(prof, sig)
            if not np.isfinite(off):
                continue
            n3 = len(prof) // 3
            parts = [(slice(0, n3)), (slice(n3, 2 * n3)), (slice(2 * n3, len(prof)))]
            offs3 = [band_offset(prof[s], sig, sign, off) for s in parts]
            rows3 = [float(np.mean(vrow[s])) for s in parts]
            # px per degree of yaw at this pole (signed)
            yw0 = C.rot_fix.get(cam, (0.0, 0.0, 0.0))
            C.rot_fix[cam] = (yw0[0] + 0.1, yw0[1], yw0[2])
            uv2, ok2 = C.project(cam, Q)
            C.rot_fix[cam] = yw0
            okb = ok & ok2
            ppd = float(np.median(uv2[okb, 0] - uv[okb, 0]) * 10) if okb.any() else np.nan
            rows.append((i, cams.index(cam), k, off, score, sign, distinct, rng[k],
                         np.degrees(np.arctan2(pv[k], pu[k])), spd[i], float(np.median(uv[ok, 0])),
                         float(np.median(vrow)), ppd, *offs3, *rows3))
r = np.array(rows)
keys = ['frame', 'cam', 'pole', 'off', 'score', 'sign', 'distinct', 'rng', 'bearing', 'speed', 'u', 'v', 'ppd',
        'off_lo', 'off_mid', 'off_hi', 'v_lo', 'v_mid', 'v_hi']
np.savez(a.out_npz, **{k: r[:, j] for j, k in enumerate(keys)})
print('measurements', len(r))
