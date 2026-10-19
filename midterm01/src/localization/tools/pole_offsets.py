#!/usr/bin/env python3
"""Measure, per camera, the horizontal pixel offset between projected map poles and the poles in the image.

For each frame and each curated map pole 4-35 m away, the pole (2 m .. min(top, 7 m) above the ground) is projected
into every camera with the current calibration; along the projected line the image rows are scanned +-40 px and
the darkest thin vertical structure (a pole against the sky) is located. A constant offset per camera points to a
yaw calibration error, an offset growing with speed to a timing offset between that camera and the trajectory.
"""
import argparse
import os
import sys

import cv2
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
from camera_model import Cameras  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('traj_csv')
ap.add_argument('out_npz')
ap.add_argument('--step', type=int, default=6)
ap.add_argument('--win', type=int, default=40)
a = ap.parse_args()
track = os.path.basename(os.path.normpath(a.track_dir))
tr = pd.read_csv(a.traj_csv)
poles = np.load(os.path.join(a.track_dir, 'poles_curated.npy'))
C = Cameras(os.path.join(a.track_dir, f'{track}_images', 'camera_info'))
Rm = Rotation.from_euler('ZYX', [-1.5, 0, 0], degrees=True).as_matrix()
cams = ['fl', 'f', 'fr']
imgdir = {c: os.path.join(a.track_dir, f'{track}_images', f'{track}_{c}_images') for c in cams}
names = {c: sorted(os.listdir(imgdir[c])) for c in cams}
tt = np.arange(len(tr)) * 0.036
spd = np.hypot(np.gradient(tr.x.values, tt), np.gradient(tr.y.values, tt))
rows = []
W = a.win
TSH = None
if os.environ.get('SDC_CAM_SYNC', '0') == '1':
    from bev import camera_time_offsets
    from scipy.ndimage import gaussian_filter1d
    dts = camera_time_offsets(a.track_dir)
    TSH = {c: gaussian_filter1d(spd, 5) * dts[c] for c in cams}
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
        for k in near:
            hs = np.linspace(2.0, min(poles[k, 4], 7.0), 24)
            if len(hs) < 2 or hs[-1] <= hs[0]:
                continue
            sh = TSH[cam][i] if TSH is not None else 0.0
            Q = np.stack([np.full(len(hs), pu[k] - sh), np.full(len(hs), pv[k]), hs - 1.5], 1) @ Rm
            uv, ok = C.project(cam, Q)
            if ok.sum() < 10 or np.ptp(uv[ok, 1]) < 40:
                continue
            if img is None:
                img = cv2.imread(os.path.join(imgdir[cam], names[cam][i]), cv2.IMREAD_GRAYSCALE).astype(np.float32)
            prof = []
            for u, v in uv[ok]:
                u, v = int(round(u)), int(round(v))
                if u - W < 0 or u + W + 1 > img.shape[1] or v < 0 or v >= img.shape[0]:
                    prof.append(None)
                    continue
                r = img[v, u - W:u + W + 1]
                prof.append(r - np.median(r))
            def locate(pl):
                pl = [q for q in pl if q is not None]
                if len(pl) < 4:
                    return np.nan, 0.0
                p = np.mean(pl, 0)
                j = int(np.argmin(p))
                if j == 0 or j == len(p) - 1:
                    return np.nan, 0.0
                den = p[j - 1] - 2 * p[j] + p[j + 1]
                sub = 0.5 * (p[j - 1] - p[j + 1]) / den if den > 0 else 0.0
                return j - W + sub, -p[j] / (np.std(p) + 1e-3)
            if sum(q is not None for q in prof) < 8:
                continue
            off_all, contrast = locate(prof)
            if not np.isfinite(off_all):
                continue
            half = len(prof) // 2      # hs ascends: first half = lower part of the pole, second = upper part
            off_lo, _ = locate(prof[:half])
            off_hi, _ = locate(prof[half:])
            j, sub = 0, 0.0
            # sensitivity: pixels per degree of camera yaw at this pole
            Qr = Q @ Rotation.from_euler('z', 0.1, degrees=True).as_matrix()
            uv2, ok2 = C.project(cam, Qr)
            ppd = np.median(np.abs(uv2[ok & ok2, 0] - uv[ok & ok2, 0])) * 10 if (ok & ok2).any() else np.nan
            rows.append((i, cams.index(cam), k, off_all, contrast, rng[k], np.degrees(np.arctan2(pv[k], pu[k])),
                         spd[i], float(np.median(uv[ok, 0])), ppd, off_lo, off_hi,
                         float(uv[ok, 1].max()), float(uv[ok, 1].min())))
r = np.array(rows)
np.savez(a.out_npz, frame=r[:, 0], cam=r[:, 1], pole=r[:, 2], off=r[:, 3], contrast=r[:, 4], rng=r[:, 5],
         bearing=r[:, 6], speed=r[:, 7], u=r[:, 8], ppd=r[:, 9], off_lo=r[:, 10], off_hi=r[:, 11],
         v_lo=r[:, 12], v_hi=r[:, 13])
print('measurements', len(r))
