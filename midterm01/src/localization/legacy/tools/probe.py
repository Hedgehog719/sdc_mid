#!/usr/bin/env python3
"""Leaderboard probing by constant offsets, to decompose the score
    S = sqrt(Mp) + k * sqrt(My),   k = 0.2, Mp = mean |dp|^2 [m^2], My = mean dtheta^2 [deg^2].

make:  probe.py make best.csv out_prefix --c 2.0 --cyaw 5.0
       writes out_prefix_{x+,x-,y+,y-,yaw+,yaw-}.csv
solve: probe.py solve --c 2 --cyaw 5 --s0 S0 --x S+ S- --y S+ S- --yaw S+ S-
       x/y probes: S = sqrt(Mp +- 2 c b + c^2) + Y   (b = mean error along that axis, Y = k sqrt(My))
       yaw probes: S = P + k sqrt(My +- 2 c by + c^2)
"""
import argparse

import numpy as np
import pandas as pd
from scipy.optimize import least_squares

K = 0.2


def make(a):
    df = pd.read_csv(a.best)
    for name, col, sgn in [('x+', 'x', 1), ('x-', 'x', -1), ('y+', 'y', 1), ('y-', 'y', -1)]:
        d = df.copy()
        d[col] += sgn * a.c
        d.to_csv(f'{a.prefix}_{name}.csv', index=False)
    for name, sgn in [('yaw+', 1), ('yaw-', -1)]:
        d = df.copy()
        d['yaw'] = (d['yaw'] + sgn * np.radians(a.cyaw) + np.pi) % (2 * np.pi) - np.pi
        d.to_csv(f'{a.prefix}_{name}.csv', index=False)


def solve(a):
    s0, c, cy = a.s0, a.c, a.cyaw
    eqs = []

    def f(v):
        Mp, My, bx, by, byaw = v
        P = np.sqrt(max(Mp, 1e-9))
        Y = K * np.sqrt(max(My, 1e-9))
        r = [P + Y - s0]
        if a.x:
            r += [np.sqrt(max(Mp + 2 * c * bx + c * c, 1e-9)) + Y - a.x[0],
                  np.sqrt(max(Mp - 2 * c * bx + c * c, 1e-9)) + Y - a.x[1]]
        if a.y:
            r += [np.sqrt(max(Mp + 2 * c * by + c * c, 1e-9)) + Y - a.y[0],
                  np.sqrt(max(Mp - 2 * c * by + c * c, 1e-9)) + Y - a.y[1]]
        if a.yaw:
            r += [P + K * np.sqrt(max(My + 2 * cy * byaw + cy * cy, 1e-9)) - a.yaw[0],
                  P + K * np.sqrt(max(My - 2 * cy * byaw + cy * cy, 1e-9)) - a.yaw[1]]
        return r
    sol = least_squares(f, [s0 ** 2 / 2, 4.0, 0.0, 0.0, 0.0])
    Mp, My, bx, by, byaw = sol.x
    print(f'pos rmse {np.sqrt(Mp):.3f} m   yaw rmse {np.sqrt(My):.3f} deg (score part {K * np.sqrt(My):.3f})')
    print(f'mean error (submission - GT): x {bx:.3f} m, y {by:.3f} m, yaw {byaw:.3f} deg   residual {np.abs(sol.fun).max():.2e}')
    print(f'pos rmse after removing mean bias: {np.sqrt(max(Mp - bx * bx - by * by, 0)):.3f} m')


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest='cmd', required=True)
    m = sub.add_parser('make')
    m.add_argument('best')
    m.add_argument('prefix')
    m.add_argument('--c', type=float, default=2.0)
    m.add_argument('--cyaw', type=float, default=5.0)
    s = sub.add_parser('solve')
    s.add_argument('--c', type=float, default=2.0)
    s.add_argument('--cyaw', type=float, default=5.0)
    s.add_argument('--s0', type=float, required=True)
    s.add_argument('--x', type=float, nargs=2)
    s.add_argument('--y', type=float, nargs=2)
    s.add_argument('--yaw', type=float, nargs=2)
    a = p.parse_args()
    make(a) if a.cmd == 'make' else solve(a)


if __name__ == '__main__':
    main()
