#!/usr/bin/env python3
"""Constant forward/lateral offset of map fixes relative to GNSS (camera longitudinal position / ground-model bias):
over the whole drive the GNSS error averages out, a median offset of forward-sharp fixes vs a GNSS+IMU-only fusion is
the fixes' own bias; it is removed from every fix. usage: fix_calib.py fixes.npz gnss_only.csv out.npz [--min_frame 600]"""
import argparse
import numpy as np
import pandas as pd
ap = argparse.ArgumentParser()
ap.add_argument('fixes'); ap.add_argument('gnss'); ap.add_argument('out')
ap.add_argument('--min_frame', type=int, default=600)
ap.add_argument('--sharp', type=float, default=0.03)
ap.add_argument('--lateral', action='store_true', help='also remove the lateral median offset')
a = ap.parse_args()
z = dict(np.load(a.fixes)); G = pd.read_csv(a.gnss)
i = z['idx'].astype(int); yw = G.yaw.values[i]
dx, dy = z['mx'] - G.x.values[i], z['my'] - G.y.values[i]
fw, lt = np.cos(yw) * dx + np.sin(yw) * dy, -np.sin(yw) * dx + np.cos(yw) * dy
m = (i >= a.min_frame) & (z['sharp_f'] >= a.sharp)
df, dl = -np.median(fw[m]), (-np.median(lt[m]) if a.lateral else 0.0)
c, s = np.cos(z['myaw']), np.sin(z['myaw'])
z['mx'] = z['mx'] + c * df - s * dl
z['my'] = z['my'] + s * df + c * dl
np.savez(a.out, **z)
print(f'fix offset vs GNSS (n {m.sum()}): forward {-df:+.3f} m, lateral {np.median(lt[m]):+.3f} m -> applied forward {df:+.3f}, lateral {dl:+.3f}')
