#!/usr/bin/env python3
"""Remove the acceleration-proportional forward error of near-range fixes, calibrated against GNSS.

near_fix uses a static camera pitch; the body pitches with longitudinal acceleration, so the near fixes' forward
error grows with it. Against a GNSS + IMU-only fusion (independent of the cameras), fit
    (near fix - GNSS) forward = c0 + c1 * accel      (moving frames >= --min_frame)
and remove c1 * accel from every fix (the constant c0 is left alone). The acceleration is the derivative of the
smoothed speed of a reference trajectory.
usage: near_accel_corr.py near.npz gnss_only.csv ref_traj.csv out.npz [--min_frame 330]
"""
import argparse

import numpy as np
import pandas as pd
from scipy.ndimage import gaussian_filter1d

ap = argparse.ArgumentParser()
ap.add_argument('near')
ap.add_argument('gnss')
ap.add_argument('ref')
ap.add_argument('out')
ap.add_argument('--min_frame', type=int, default=330)
a = ap.parse_args()
F, G = pd.read_csv(a.ref), pd.read_csv(a.gnss)
yw = F.yaw.values
xs, ys = gaussian_filter1d(F.x.values, 3), gaussian_filter1d(F.y.values, 3)
v = np.hypot(np.gradient(xs), np.gradient(ys)) * 10
acc = gaussian_filter1d(np.gradient(v), 10) * 10
z = dict(np.load(a.near))
i = z['idx'].astype(int)
fw = np.cos(yw[i]) * (z['mx'] - G.x.values[i]) + np.sin(yw[i]) * (z['my'] - G.y.values[i])
m = (i >= a.min_frame) & (v[i] > 1.0)
c0, c1 = np.linalg.lstsq(np.stack([np.ones(m.sum()), acc[i[m]]], 1), fw[m], rcond=None)[0]
corr = c1 * acc[i]
z['mx'] = z['mx'] - np.cos(z['myaw']) * corr
z['my'] = z['my'] - np.sin(z['myaw']) * corr
np.savez(a.out, **z)
print(f'{a.near}: (near - GNSS) forward = {c0:+.3f} + {c1:+.3f} * accel ({m.sum()} moving fixes); accel term removed -> {a.out}')
