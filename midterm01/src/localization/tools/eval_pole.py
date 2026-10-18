#!/usr/bin/env python3
"""Evaluate pole_match / bev_match fixes vs a reference trajectory (fwd / lat errors, by quality threshold)."""
import sys

import numpy as np
import pandas as pd

z = np.load(sys.argv[1])
gt = pd.read_csv(sys.argv[2]).iloc[z['idx']]
yw = gt.yaw.values


def ev(X, Y):
    dx, dy = X - gt.x.values, Y - gt.y.values
    return np.cos(yw) * dx + np.sin(yw) * dy, -np.sin(yw) * dx + np.cos(yw) * dy


pf, pl = ev(z['px'], z['py'])
mf, ml = ev(z['mx'], z['my'])
r = lambda v, m: np.sqrt(np.mean(v[m] ** 2))  # noqa: E731
a = np.ones(len(pf), bool)
print(f'prior      n={len(pf)} fwd rms {r(pf, a):.2f} lat rms {r(pl, a):.2f}')
q = z['peak']
for t in [0, 50, 75]:
    m = q >= np.percentile(q, t)
    print(f'meas peak>=p{t:<2d} n={m.sum():4d} fwd rms {r(mf, m):.2f} med {np.median(np.abs(mf[m])):.2f} | '
          f'lat rms {r(ml, m):.2f} med {np.median(np.abs(ml[m])):.2f}')
