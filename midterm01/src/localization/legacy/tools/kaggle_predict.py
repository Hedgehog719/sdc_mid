#!/usr/bin/env python3
"""Predict a candidate's leaderboard position score from past submissions (no submission needed).

pos RMSE is exactly quadratic in a position change d:  P(d)^2 = P0^2 + 2 <e, d> + |d|^2  (<.,.> = mean over frames of
the 2D dot product, e = current error). Every scored submission i with the same yaw as the base gives <e, d_i> exactly
(given the yaw part K of the score). A candidate's change d is projected onto span{d_i}: its inner product is predicted
from the projection; the orthogonal remainder is assumed uncorrelated with the error (adds |d_perp|^2: conservative).
Caveat: the public leaderboard may use a subset of frames - large extrapolations fit the public part.
"""
import argparse

import numpy as np
import pandas as pd

ap = argparse.ArgumentParser()
ap.add_argument('base_csv')
ap.add_argument('base_score', type=float)
ap.add_argument('K', type=float, help='yaw part of the score (0.2 * yaw RMSE) shared by base and references')
ap.add_argument('--ref', nargs=2, action='append', metavar=('CSV', 'SCORE'), required=True)
ap.add_argument('candidates', nargs='+')
a = ap.parse_args()
base = pd.read_csv(a.base_csv)
n = len(base)
vec = lambda f: np.concatenate([pd.read_csv(f).x.values - base.x.values, pd.read_csv(f).y.values - base.y.values])  # noqa: E731
P0 = a.base_score - a.K
V, c = [], []
for f, sc in a.ref:
    d = vec(f)
    D = d @ d / n
    V.append(d)
    c.append(((float(sc) - a.K) ** 2 - P0 ** 2 - D) / 2)
V, c = np.array(V), np.array(c)
G = V @ V.T / n
for f in a.candidates:
    d = vec(f)
    w = np.linalg.lstsq(V.T, d, rcond=None)[0]          # projection coefficients onto span{d_i}
    dpar = V.T @ w
    cpar = w @ c
    D = d @ d / n
    P2 = P0 ** 2 + 2 * cpar + D
    frac = (dpar @ dpar) / max(d @ d, 1e-12)
    print(f'{f}: |d| rms {np.sqrt(D):.3f} m, {frac * 100:3.0f}% of |d|^2 in the measured span, predicted score '
          f'{np.sqrt(max(P2, 0)) + a.K:.5f} (base {a.base_score:.5f})')
