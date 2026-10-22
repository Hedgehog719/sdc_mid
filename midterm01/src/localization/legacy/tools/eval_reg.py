#!/usr/bin/env python3
"""Offline evaluation of registration against a pseudo ground truth trajectory (fused CSV).

Reports the error of the prior P and of the registration T in the GT vehicle frame (fwd, lat, yaw).
With --prior_noise in register.py this measures whether matching pulls a perturbed prior back.
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(__file__))
from common import wrap, yaw_of  # noqa: E402


def errors(M, gt):
    d = M[:, :2, 3] - gt[['x', 'y']].values
    yw = gt.yaw.values
    fwd = np.cos(yw) * d[:, 0] + np.sin(yw) * d[:, 1]
    lat = -np.sin(yw) * d[:, 0] + np.cos(yw) * d[:, 1]
    dyaw = np.degrees(wrap(yaw_of(M) - yw))
    return fwd, lat, dyaw


def summary(name, fwd, lat, dyaw, mask=None):
    m = np.ones(len(fwd), bool) if mask is None else mask
    r = lambda v: np.sqrt(np.mean(v[m] ** 2))  # noqa: E731
    med = np.median(np.hypot(fwd[m], lat[m]))
    print(f'{name:>10}: n={m.sum():4d} fwd {r(fwd):.2f} lat {r(lat):.2f} pos {np.sqrt(r(fwd)**2 + r(lat)**2):.2f} '
          f'(median {med:.2f}) yaw {r(dyaw):.2f} deg')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('npz')
    p.add_argument('gt_csv')
    a = p.parse_args()
    z = np.load(a.npz)
    gt = pd.read_csv(a.gt_csv).iloc[z['idx']]
    summary('prior', *errors(z['P'], gt))
    f, l, y = errors(z['T'], gt)
    summary('matched', f, l, y)
    good = np.abs(np.degrees(wrap(yaw_of(z['T']) - yaw_of(z['P'])))) < 6
    good &= np.linalg.norm(z['T'][:, :2, 3] - z['P'][:, :2, 3], axis=1) < 3
    summary('gated', f, l, y, good)


if __name__ == '__main__':
    main()
