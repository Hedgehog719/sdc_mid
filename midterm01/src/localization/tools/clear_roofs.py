#!/usr/bin/env python3
"""Remove vehicles standing on the road (car roofs) from the map along the whole driven path.

Where a vehicle hid the ground completely while mapping (e.g. at the Track2 start: no ground under the car, only its
roof at +1.4..2.8 m), every ground estimate (per-cell minimum, grey opening) takes the roof as ground, so the road
under the vehicle tilts by up to 13 deg. Here, in path coordinates (s along the path, l left), per 2 m cell the
10th-percentile height is taken; per --win m of path a lower-envelope plane is fitted to the cells with |l| < --fit_half
(iterative: cells more than --env above the plane are dropped, roofs are always above the road); then points with
|l| < --road_half between plane + --clear_h and + --max_h are deleted (vehicles, people; poles and signs stand beside the road).
Writes map_<tag_out>.npy + intensity, and prints the cleared segments.

usage: clear_roofs.py <track_dir> <traj_csv> --tag_in clean --tag_out clean12
"""
import argparse
import os

import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('traj_csv')
ap.add_argument('--tag_in', default='clean')
ap.add_argument('--tag_out', required=True)
ap.add_argument('--win', type=float, default=12.0, help='m of path per plane')
ap.add_argument('--fit_half', type=float, default=8.0)
ap.add_argument('--road_half', type=float, default=5.0)
ap.add_argument('--clear_h', type=float, default=1.0)
ap.add_argument('--max_h', type=float, default=4.0, help='keep points higher than road + max_h (signs, bridges, tree crowns)')
ap.add_argument('--env', type=float, default=0.25)
ap.add_argument('--back', type=float, default=30.0, help='m before the start along the initial heading')
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
qs = np.arange(-a.back, s_cum[-1] + 5, 0.5)
px = np.where(qs < 0, x[0] + qs * hx, np.interp(np.clip(qs, 0, None), s_cum, x))
py = np.where(qs < 0, y[0] + qs * hy, np.interp(np.clip(qs, 0, None), s_cum, y))
gx, gy = np.gradient(px), np.gradient(py)
gn = np.hypot(gx, gy)
gx, gy = np.where(gn > 0, gx / np.maximum(gn, 1e-9), np.nan), np.where(gn > 0, gy / np.maximum(gn, 1e-9), np.nan)
gx, gy = pd.Series(gx).ffill().bfill().values, pd.Series(gy).ffill().bfill().values
xyz = np.asarray(P, dtype=np.float64)
d, j = cKDTree(np.stack([px, py], 1)).query(xyz[:, :2], distance_upper_bound=a.fit_half + 2)
inr = np.isfinite(d)
idx = np.flatnonzero(inr)
jj = j[inr]
sp = qs[jj] + gx[jj] * (xyz[idx, 0] - px[jj]) + gy[jj] * (xyz[idx, 1] - py[jj])
lp = -gy[jj] * (xyz[idx, 0] - px[jj]) + gx[jj] * (xyz[idx, 1] - py[jj])
zp = xyz[idx, 2]
cs = np.floor((sp + a.back) / 2.0).astype(int)
cl = np.floor((lp + a.fit_half + 2) / 2.0).astype(int)
nl = cl.max() + 1
key = cs * nl + cl
o = np.lexsort((zp, key))
u, first, cnt = np.unique(key[o], return_index=True, return_counts=True)
gz = zp[o][first + (0.1 * (cnt - 1)).astype(int)]
gs = (u // nl + 0.5) * 2.0 - a.back
gl = (u % nl + 0.5) * 2.0 - a.fit_half - 2
ok_c = (np.abs(gl) < a.fit_half) & (cnt >= 5)
# per window: lower-envelope plane z = c0 + c1 (s - s_mid) + c2 l
nw = int(np.ceil((qs[-1] + a.back) / a.win)) + 1
planes = np.full((nw, 3), np.nan)
wid_c = np.floor((gs + a.back) / a.win).astype(int)
for w in range(nw):
    m = ok_c & (np.abs(wid_c - w) <= 1)     # 3 windows of cells for a stable fit
    if m.sum() < 8:
        continue
    sm = (w + 0.5) * a.win - a.back
    A = np.stack([np.ones(m.sum()), gs[m] - sm, gl[m]], 1)
    z = gz[m]
    keep = np.ones(len(z), bool)
    for _ in range(6):
        co = np.linalg.lstsq(A[keep], z[keep], rcond=None)[0]
        r = z - A @ co
        nk = r < a.env
        if nk.sum() < 6 or (nk == keep).all():
            break
        keep = nk
    planes[w] = co
wid = np.clip(np.floor((sp + a.back) / a.win).astype(int), 0, nw - 1)
sm = (wid + 0.5) * a.win - a.back
pl = planes[wid]
zref = pl[:, 0] + pl[:, 1] * (sp - sm) + pl[:, 2] * lp
above = np.isfinite(zref) & (np.abs(lp) < a.road_half) & (zp > zref + a.clear_h) & (zp < zref + a.max_h)
drop = np.zeros(len(P), bool)
drop[idx[above]] = True
print(f'cleared {above.sum()} points ({above.sum() / len(P) * 100:.3f} %) road + {a.clear_h}..{a.max_h} m within {a.road_half} m of the path')
# report segments (per 2 m of path with > 200 cleared points)
hs = np.bincount(np.clip(((sp[above] + a.back) / 2).astype(int), 0, None), minlength=int((qs[-1] + a.back) / 2) + 2)
seg = []
for k in np.flatnonzero(hs > 200):
    s0 = k * 2 - a.back
    if seg and s0 - seg[-1][1] <= 4:
        seg[-1][1] = s0 + 2
        seg[-1][2] += hs[k]
    else:
        seg.append([s0, s0 + 2, hs[k]])
fr_of = lambda s: int(np.searchsorted(s_cum, max(s, 0)))  # noqa: E731
for s0, s1, n in seg:
    print(f'   s {s0:7.0f}..{s1:7.0f} m (frames {fr_of(s0)}-{fr_of(s1)}): {n} points')
np.save(os.path.join(a.track_dir, f'removed_{a.tag_out}_idx.npy'), np.flatnonzero(drop))   # indices into map_<tag_in>
np.save(os.path.join(a.track_dir, f'map_{a.tag_out}.npy'), np.asarray(P)[~drop])
np.save(os.path.join(a.track_dir, f'map_intensity_{a.tag_out}.npy'), np.asarray(I)[~drop])
print('saved', f'map_{a.tag_out}.npy')
if os.environ.get('ROOF_STATS'):
    lh = np.histogram(lp[above], bins=np.arange(-5, 5.01, 1))[0]
    zh = np.histogram((zp - zref)[above], bins=[0.5, 1, 1.5, 2, 2.5, 3, 4])[0]
    print('cleared points by lateral offset l (m, left +):', dict(zip(np.arange(-5, 5).tolist(), lh.tolist())))
    print('by height above road (m):', dict(zip(['.5-1', '1-1.5', '1.5-2', '2-2.5', '2.5-3', '3-4'], zh.tolist())))
    occ = np.bincount(np.clip(((sp[above] + a.back) / 2).astype(int), 0, None))
    print(f'2 m path cells with > 50 cleared points: {(occ > 50).sum()} of {len(occ)}')
