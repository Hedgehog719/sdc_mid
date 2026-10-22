#!/usr/bin/env python3
"""Level an uplifted map plate back onto the surrounding road (vertical splice).

Assumption (user): the raised region is ordinary road at the same grade as the trusted road next to it. In path
coordinates (s along the driven path from the start, l left) the ground (low percentile per 2 m cell) is compared
with a plane fitted to trusted cells (s in --trust); cells of the plate (s < --seam) get the full correction, a
band --seam .. --seam + --band blends linearly to zero, so the two regions join without a step. Every point of a
cell is shifted by the same dz (objects keep their height above the ground). Writes map_<tag_out>.npy.
"""
import argparse
import os

import numpy as np
import pandas as pd
from scipy.ndimage import median_filter
from scipy.spatial import cKDTree

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('traj_csv')
ap.add_argument('--tag_in', default='clean')
ap.add_argument('--tag_out', default='clean9')
ap.add_argument('--seam', type=float, default=8.0, help='m along the path: plate ends here')
ap.add_argument('--band', type=float, default=7.0, help='m of linear blending after the seam')
ap.add_argument('--trust', default='10:40', help='s range (m) of the trusted road for the reference plane')
ap.add_argument('--back', type=float, default=30.0, help='m before the start that belong to the plate')
ap.add_argument('--half_width', type=float, default=14.0)
ap.add_argument('--road_half', type=float, default=8.0, help='cells used for the plane fit / full correction')
ap.add_argument('--max_dz', type=float, default=2.5)
ap.add_argument('--shift_fwd', type=float, default=0.0, help='m: horizontal shift of the plate along the path direction')
ap.add_argument('--shift_lat', type=float, default=0.0, help='m: horizontal shift of the plate to the left')
ap.add_argument('--hseam', type=float, default=10.0, help='horizontal plate ends here (m along the path)')
ap.add_argument('--hband', type=float, default=15.0)
ap.add_argument('--cell', type=float, default=2.0)
ap.add_argument('--clear_h', type=float, default=0.0,
                help='>0: instead of shifting, delete plate points (s < seam, |l| < road_half) higher than the reference '
                     'road plane + clear_h (the "plate" can be an object standing on the road with no ground under it)')
a = ap.parse_args()

P = np.load(os.path.join(a.track_dir, f'map_{a.tag_in}.npy'), mmap_mode='r')
I = np.load(os.path.join(a.track_dir, f'map_intensity_{a.tag_in}.npy'), mmap_mode='r')
tr = pd.read_csv(a.traj_csv)
x, y = tr.x.values, tr.y.values
s_cum = np.r_[0, np.cumsum(np.hypot(np.diff(x), np.diff(y)))]
t0, t1 = (float(v) for v in a.trust.split(':'))
# dense path samples, extended backwards along the initial heading for the region behind the start
k0 = int(np.searchsorted(s_cum, 3.0))
hx, hy = x[k0] - x[0], y[k0] - y[0]
hn = np.hypot(hx, hy)
hx, hy = hx / hn, hy / hn
qs = np.arange(-a.back, a.seam + a.band + t1 + 5, 0.5)
px = np.where(qs < 0, x[0] + qs * hx, np.interp(np.clip(qs, 0, None), s_cum, x))
py = np.where(qs < 0, y[0] + qs * hy, np.interp(np.clip(qs, 0, None), s_cum, y))
gx, gy = np.gradient(px), np.gradient(py)
gn = np.hypot(gx, gy)
gx, gy = gx / gn, gy / gn
xyz = np.asarray(P[:, :3], dtype=np.float64)
d, j = cKDTree(np.stack([px, py], 1)).query(xyz[:, :2], distance_upper_bound=a.half_width)
inr = np.isfinite(d)
idx = np.flatnonzero(inr)
jj = j[inr]
sp = qs[jj] + gx[jj] * (xyz[idx, 0] - px[jj]) + gy[jj] * (xyz[idx, 1] - py[jj])
lp = -gy[jj] * (xyz[idx, 0] - px[jj]) + gx[jj] * (xyz[idx, 1] - py[jj])
zp = xyz[idx, 2]
# ground per cell: 10th percentile
cs = np.floor((sp + a.back) / a.cell).astype(int)
cl = np.floor((lp + a.half_width) / a.cell).astype(int)
ns, nl = cs.max() + 1, cl.max() + 1
key = cs * nl + cl
o = np.lexsort((zp, key))
ks, zs = key[o], zp[o]
u, first, cnt = np.unique(ks, return_index=True, return_counts=True)
G = np.full(ns * nl, np.nan)
G[u] = zs[first + (0.1 * (cnt - 1)).astype(int)]
G = G.reshape(ns, nl)
s_c = (np.arange(ns) + 0.5) * a.cell - a.back
l_c = (np.arange(nl) + 0.5) * a.cell - a.half_width
SS, LL = np.meshgrid(s_c, l_c, indexing='ij')
trust = np.isfinite(G) & (SS >= t0) & (SS <= t1) & (np.abs(LL) < a.road_half)
A = np.stack([np.ones(trust.sum()), SS[trust], LL[trust]], 1)
coef = np.linalg.lstsq(A, G[trust], rcond=None)[0]
r = G[trust] - A @ coef
keep = np.abs(r) < 3 * 1.4826 * np.median(np.abs(r))
coef = np.linalg.lstsq(A[keep], G[trust][keep], rcond=None)[0]
target = coef[0] + coef[1] * SS + coef[2] * LL
dev = G - target
# fill holes with the median of neighbours, then smooth
devf = np.where(np.isfinite(dev), dev, 0.0)
devm = median_filter(devf, size=3)
devf = np.where(np.isfinite(dev), devf, devm)
w = np.clip(1 - (SS - a.seam) / a.band, 0, 1)          # 1 in the plate, linear to 0 across the band
wl = np.clip((a.half_width - np.abs(LL)) / (a.half_width - a.road_half), 0, 1)   # taper outside the road
corr = -np.clip(devf, -a.max_dz, a.max_dz) * w * wl
corr[~np.isfinite(G)] = 0.0
print(f'reference plane: z = {coef[0]:.3f} + {coef[1] * 100:.3f} % s + {coef[2] * 100:.3f} % l (trusted cells {trust.sum()})')
for sv in (-20, -10, -4, 0, 2, 4, 6, 8, 10, 12, 14):
    ii = int((sv + a.back) / a.cell)
    if 0 <= ii < ns:
        row = dev[ii][np.abs(l_c) < 6]
        print(f'  s {sv:+4d} m: ground deviation (|l|<6 m) median {np.nanmedian(row):+.3f} m -> correction {np.nanmedian(corr[ii][np.abs(l_c) < 6]):+.3f}')
dz = corr[cs, cl]
out = np.asarray(P).copy()
keep_pts = np.ones(len(out), bool)
if a.clear_h > 0:
    dz[:] = 0.0
    above = (sp < a.seam) & (np.abs(lp) < a.road_half) & (zp > coef[0] + coef[1] * sp + coef[2] * lp + a.clear_h)
    keep_pts[idx[above]] = False
    print(f'cleared {above.sum()} points above road + {a.clear_h} m in the plate (s < {a.seam}, |l| < {a.road_half})')
out[idx, 2] = (out[idx, 2] + dz).astype(out.dtype)
if a.shift_fwd or a.shift_lat:   # horizontal plate shift, linear to zero across the horizontal band
    wh = np.clip(1 - (sp - a.hseam) / a.hband, 0, 1) * np.clip((a.half_width - np.abs(lp)) / (a.half_width - a.road_half), 0, 1)
    out[idx, 0] = (out[idx, 0] + wh * (a.shift_fwd * gx[jj] - a.shift_lat * gy[jj])).astype(out.dtype)
    out[idx, 1] = (out[idx, 1] + wh * (a.shift_fwd * gy[jj] + a.shift_lat * gx[jj])).astype(out.dtype)
    print(f'horizontal plate shift fwd {a.shift_fwd:+.2f} lat {a.shift_lat:+.2f} m on {(wh > 0.01).sum()} points')
moved = np.abs(dz) > 0.02
print(f'points corrected: {moved.sum()} ({moved.sum() / len(P) * 100:.2f} %), max |dz| {np.abs(dz).max():.2f} m')
np.save(os.path.join(a.track_dir, f'map_{a.tag_out}.npy'), out[keep_pts])
np.save(os.path.join(a.track_dir, f'map_intensity_{a.tag_out}.npy'), np.asarray(I)[keep_pts])
print('saved', f'map_{a.tag_out}.npy')
