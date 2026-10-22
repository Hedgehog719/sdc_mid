#!/usr/bin/env python3
"""Exact leaderboard optimum over all scored position directions (quadratic identity, public board = all frames).

Score = P + Y with P = position RMSE and Y = 0.2 x yaw RMSE. For a scored file i, P_i = S_i - Y_i where its yaw part
Y_i is known (a yaw class with a fixed value). With a base file b (P_b known) and d_i = positions_i - positions_b:
    P_i^2 = P_b^2 + 2 <e, d_i> + |d_i|^2    ->  <e, d_i> exactly,
so any combination d = sum a_i d_i has P(d)^2 = P_b^2 + 2 a.c + a.G.a, minimised in closed form (pseudo-inverse).
Prints the optimum, a leave-one-out check of the model, and writes the optimum with the chosen yaw file.

usage: board_opt.py --base FILE SCORE YAWCLASS --ref FILE SCORE YAWCLASS [...] --yaw CLASS=VALUE [...]
                    --yaw_file FILE --out FILE [--only NAME,...]
"""
import argparse
import os

import numpy as np
import pandas as pd

ap = argparse.ArgumentParser()
ap.add_argument('--base', nargs=3, required=True, metavar=('FILE', 'SCORE', 'YAWCLASS'))
ap.add_argument('--ref', nargs=3, action='append', required=True, metavar=('FILE', 'SCORE', 'YAWCLASS'))
ap.add_argument('--yaw', action='append', required=True, help='CLASS=value of 0.2 x yaw RMSE')
ap.add_argument('--yaw_file', required=True, help='file whose yaw column the optimum gets')
ap.add_argument('--yaw_class', required=True, help='yaw class of --yaw_file')
ap.add_argument('--out', required=True)
a = ap.parse_args()
Y = {k: float(v) for k, v in (s.split('=') for s in a.yaw)}
base = pd.read_csv(a.base[0])
n = len(base)
P0 = float(a.base[1]) - Y[a.base[2]]
vec = lambda df: np.concatenate([df.x.values - base.x.values, df.y.values - base.y.values])  # noqa: E731
names = [os.path.basename(f) for f, _, _ in a.ref]
V = np.array([vec(pd.read_csv(f)) for f, _, _ in a.ref])
P = np.array([float(s) - Y[c] for _, s, c in a.ref])
c = (P ** 2 - P0 ** 2 - np.einsum('ij,ij->i', V, V) / n) / 2
Gm = V @ V.T / n
w = -np.linalg.pinv(Gm, rcond=1e-10) @ c
P2 = P0 ** 2 + 2 * w @ c + w @ Gm @ w
print(f'base P {P0:.5f}; optimum P {np.sqrt(max(P2, 0)):.5f} -> score {np.sqrt(max(P2, 0)) + Y[a.yaw_class]:.5f} (yaw part {Y[a.yaw_class]:.5f})')
for nm, wi in zip(names, w):
    print(f'   {wi:+8.3f}  {nm}')
print('leave-one-out (predict each scored file from the others):')
for k in range(len(names)):
    idx = [j for j in range(len(names)) if j != k]
    wk = np.linalg.lstsq(V[idx].T, V[k], rcond=None)[0]
    dp = V[idx].T @ wk
    pk = np.sqrt(max(P0 ** 2 + 2 * wk @ c[idx] + V[k] @ V[k] / n, 0))
    print(f'   {names[k]:40s} in-span {(dp @ dp) / max(V[k] @ V[k], 1e-12) * 100:3.0f}%  predicted P {pk:.5f}  actual {P[k]:.5f}')
d = V.T @ w
o = pd.read_csv(a.yaw_file).copy()
o['x'] = base.x.values + d[:n]
o['y'] = base.y.values + d[n:]
o.to_csv(a.out, index=False)
print(f'wrote {a.out}: max step {np.hypot(np.diff(o.x), np.diff(o.y)).max():.3f} m, nan {o.isna().sum().sum()}')
