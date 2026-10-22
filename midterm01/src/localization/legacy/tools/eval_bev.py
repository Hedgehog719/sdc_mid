#!/usr/bin/env python3
"""Evaluate bev_match.py output against a reference trajectory (pseudo-GT): prior vs measurement errors."""
import argparse

import numpy as np
import pandas as pd


def err(x, y, yaw, gt):
    d = np.stack([x - gt.x.values, y - gt.y.values], 1)
    c, s = np.cos(gt.yaw.values), np.sin(gt.yaw.values)
    fwd = c * d[:, 0] + s * d[:, 1]
    lat = -s * d[:, 0] + c * d[:, 1]
    dy = np.degrees((yaw - gt.yaw.values + np.pi) % (2 * np.pi) - np.pi)
    return fwd, lat, dy


def line(name, f, l, y, m):
    r = lambda v: np.sqrt(np.mean(v[m] ** 2)) if m.any() else np.nan  # noqa: E731
    print(f'{name:>14}: n={m.sum():4d} fwd {r(f):.2f} lat {r(l):.2f} yaw {r(y):.2f} deg  '
          f'median|pos| {np.median(np.hypot(f[m], l[m])) if m.any() else np.nan:.2f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('npz')
    ap.add_argument('gt_csv')
    ap.add_argument('--sharp', type=float, nargs='*', default=[0.0, 0.02, 0.05])
    a = ap.parse_args()
    z = np.load(a.npz)
    gt = pd.read_csv(a.gt_csv).iloc[z['idx']]
    all_ = np.ones(len(z['idx']), bool)
    line('prior', *err(z['px'], z['py'], z['pyaw'], gt), all_)
    f, l, y = err(z['mx'], z['my'], z['myaw'], gt)
    for t in a.sharp:
        line(f'meas sharp>{t}', f, l, y, z['sharp'] > t)
    print('peak median %.3f sharp median %.3f' % (np.median(z['peak']), np.median(z['sharp'])))


if __name__ == '__main__':
    main()
