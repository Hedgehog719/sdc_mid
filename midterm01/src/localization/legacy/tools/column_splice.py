#!/usr/bin/env python3
"""Lift (or lower) a mis-registered map chunk back onto the road before and after it, column by column.

Track1 start: at s = +2..+20 m the whole road (and its curbs) sits up to 1.2 m too low, tilted, then returns -
a pit no camera image shows. In path coordinates (s, l), per 1 x 1 m column the ground is the 10th percentile of its
points; the reference is, per lateral column, the linear interpolation in s between the ground before (--before
s range) and after (--after) the chunk. The correction ref - ground is median-filtered (roof cells without ground
become their neighbours), smoothed, limited to s in --region and |l| < --half (taper over --taper m), and added to
EVERY point of the column (bilinear), so curbs, poles and road edges move with their ground and keep their shape.

usage: column_splice.py <track_dir> <traj_csv> --tag_in clean --tag_out clean14 --before=-8:0 --after 18:26 --region 0:22
"""
import argparse
import os

import numpy as np
import pandas as pd
from scipy.ndimage import gaussian_filter, map_coordinates, median_filter
from scipy.spatial import cKDTree

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('traj_csv')
ap.add_argument('--tag_in', default='clean')
ap.add_argument('--tag_out', required=True)
ap.add_argument('--before', default='-8:0')
ap.add_argument('--after', default='18:26')
ap.add_argument('--region', default='0:22', help='s range that gets corrected (zero outside, blended over 2 m)')
ap.add_argument('--half', type=float, default=12.0)
ap.add_argument('--taper', type=float, default=3.0)
ap.add_argument('--back', type=float, default=30.0)
a = ap.parse_args()

sfx = f'_{a.tag_in}' if a.tag_in else ''
P = np.load(os.path.join(a.track_dir, f'map{sfx}.npy'), mmap_mode='r')
I = np.load(os.path.join(a.track_dir, f'map_intensity{sfx}.npy'), mmap_mode='r')
tr = pd.read_csv(a.traj_csv)
x, y = tr.x.values, tr.y.values
s_cum = np.r_[0, np.cumsum(np.hypot(np.diff(x), np.diff(y)))]
k0 = int(np.searchsorted(s_cum, 3.0))
hx, hy = x[k0] - x[0], y[k0] - y[0]
hn = np.hypot(hx, hy)
hx, hy = hx / hn, hy / hn
b0, b1 = (float(v) for v in a.before.split(':'))
f0, f1 = (float(v) for v in a.after.split(':'))
r0, r1 = (float(v) for v in a.region.split(':'))
qs = np.arange(min(b0, -a.back), f1 + 10, 0.5)
px = np.where(qs < 0, x[0] + qs * hx, np.interp(np.clip(qs, 0, None), s_cum, x))
py = np.where(qs < 0, y[0] + qs * hy, np.interp(np.clip(qs, 0, None), s_cum, y))
gx, gy = np.gradient(px), np.gradient(py)
gn = np.hypot(gx, gy)
gx, gy = gx / gn, gy / gn
xyz = np.asarray(P, dtype=np.float64)
d, j = cKDTree(np.stack([px, py], 1)).query(xyz[:, :2], distance_upper_bound=a.half + 2)
idx = np.flatnonzero(np.isfinite(d))
jj = j[idx]
sp = qs[jj] + gx[jj] * (xyz[idx, 0] - px[jj]) + gy[jj] * (xyz[idx, 1] - py[jj])
lp = -gy[jj] * (xyz[idx, 0] - px[jj]) + gx[jj] * (xyz[idx, 1] - py[jj])
zp = xyz[idx, 2]
S0, L0 = qs[0], -a.half - 2
cs = np.floor(sp - S0).astype(int)
cl = np.floor(lp - L0).astype(int)
ns, nl = cs.max() + 1, cl.max() + 1
key = cs * nl + cl
o = np.lexsort((zp, key))
u, first, cnt = np.unique(key[o], return_index=True, return_counts=True)
G = np.full(ns * nl, np.nan)
good = cnt >= 5
G[u[good]] = zp[o][first[good] + (0.1 * (cnt[good] - 1)).astype(int)]
G = G.reshape(ns, nl)
s_c = S0 + np.arange(ns) + 0.5
l_c = L0 + np.arange(nl) + 0.5
bef = np.nanmedian(G[(s_c >= b0) & (s_c <= b1)], 0)
aft = np.nanmedian(G[(s_c >= f0) & (s_c <= f1)], 0)
w = np.clip((s_c - (b0 + b1) / 2) / ((f0 + f1) / 2 - (b0 + b1) / 2), 0, 1)[:, None]
ref = bef[None] * (1 - w) + aft[None] * w
dz = ref - G
fin = np.isfinite(dz)
dzf = median_filter(np.where(fin, dz, 0.0), size=5)
dzm = np.where(fin, dz, dzf)
dzm = np.where(np.abs(dzm - dzf) > 0.3, dzf, dzm)          # roof / hole cells -> neighbours
dzm = gaussian_filter(dzm, 1.0)
ws = np.clip(np.minimum(s_c - r0, r1 - s_c) / 2.0 + 0.5, 0, 1)[:, None]
wl = np.clip((a.half - np.abs(l_c)) / a.taper, 0, 1)[None]
corr = np.nan_to_num(dzm) * ws * wl
print('correction (m) per 2 m of s (rows) and 4 m of l (cols', l_c[::4].round(1).tolist(), '):')
for k in range(0, ns, 2):
    if r0 - 3 <= s_c[k] <= r1 + 3:
        print(f'  s {s_c[k]:+6.1f}: ' + ' '.join(f'{v:+5.2f}' for v in corr[k, ::4]))
dzp = map_coordinates(corr, [sp - S0 - 0.5, lp - L0 - 0.5], order=1, mode='nearest')
out = np.asarray(P).copy()
out[idx, 2] = (out[idx, 2] + dzp).astype(out.dtype)
print(f'moved {(np.abs(dzp) > 0.02).sum()} points, max |dz| {np.abs(dzp).max():.2f} m')
np.save(os.path.join(a.track_dir, f'map_{a.tag_out}.npy'), out)
np.save(os.path.join(a.track_dir, f'map_intensity_{a.tag_out}.npy'), np.asarray(I))
print('saved', f'map_{a.tag_out}.npy')
