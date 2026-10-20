#!/usr/bin/env python3
"""Anchor the forward position of a fix set to the near-range image fixes, segment by segment.

Map-derived fix sources disagree in their forward reference (T2, vs GNSS: lane -0.07 m, poles -0.10 m, near-range
+0.03 m), while each keeps a precise frame-to-frame shape. The near-range fixes (road markings 1-12 m ahead,
near_fix.py) are noisy per frame in forward but their segment medians are the best absolute forward reference. For
every fix, the median forward difference (fix - near fix at nearby frames, both relative to a reference trajectory)
over +-win frames is removed. Lateral is left unchanged (all map sources agree laterally).

usage: fix_anchor.py fixes.npz near.npz ref.csv out.npz [--win 150 --gap 3 --min_n 8]
"""
import argparse

import numpy as np
import pandas as pd

ap = argparse.ArgumentParser()
ap.add_argument('fixes')
ap.add_argument('near')
ap.add_argument('ref')
ap.add_argument('out')
ap.add_argument('--win', type=int, default=150)
ap.add_argument('--gap', type=int, default=3, help='max frame gap for a fix / near-fix pair')
ap.add_argument('--min_n', type=int, default=8)
a = ap.parse_args()
z = dict(np.load(a.fixes))
nz = dict(np.load(a.near))
T = pd.read_csv(a.ref)


def rel(d):
    i = d['idx'].astype(int)
    yw = T.yaw.values[i]
    dx, dy = d['mx'] - T.x.values[i], d['my'] - T.y.values[i]
    return i, np.cos(yw) * dx + np.sin(yw) * dy


fi, ff = rel(z)
ni, nf = rel(nz)
o = np.argsort(ni)
ni, nf = ni[o], nf[o]
# pairwise differences fix - near at nearby frames
k = np.clip(np.searchsorted(ni, fi), 1, len(ni) - 1)
k = np.where(np.abs(ni[k - 1] - fi) < np.abs(ni[k] - fi), k - 1, k)
ok = np.abs(ni[k] - fi) <= a.gap
diff = np.where(ok, ff - nf[k], np.nan)
corr = np.zeros(len(fi))
for j in range(len(fi)):
    m = ok & (np.abs(fi - fi[j]) <= a.win)
    if m.sum() >= a.min_n:
        corr[j] = np.nanmedian(diff[m])
c, s = np.cos(z['myaw']), np.sin(z['myaw'])
z['mx'] = z['mx'] - c * corr
z['my'] = z['my'] - s * corr
np.savez(a.out, **z)
print(f'{a.fixes}: {ok.sum()} pairs with near fixes; forward correction median {np.median(corr):+.3f} m '
      f'(p10 {np.percentile(corr, 10):+.3f}, p90 {np.percentile(corr, 90):+.3f}) -> {a.out}')
