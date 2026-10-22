#!/usr/bin/env python3
"""Shift a trajectory CSV in time by tau seconds (resampling along itself) for lag probing / correction.

Probe math: with velocity v, shifting by tau gives Mp' = Mp + 2 tau E[e.v] + tau^2 E[|v|^2]; two probes
(+tau, -tau) plus the base score give E[e.v], hence the optimal lag tau* = -E[e.v] / E[|v|^2].
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(__file__))
from common import ply_frames, wrap  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('track_dir')
    ap.add_argument('in_csv')
    ap.add_argument('out_csv')
    ap.add_argument('--tau', type=float, required=True)
    a = ap.parse_args()
    ts, _ = ply_frames(os.path.join(a.track_dir, 'raw_ply'))
    t = (ts - ts[0]) * 1e-9
    d = pd.read_csv(a.in_csv)
    yaw = np.unwrap(d.yaw.values)
    tq = t + a.tau
    out = d.copy()
    out['x'] = np.interp(tq, t, d.x)
    out['y'] = np.interp(tq, t, d.y)
    out['yaw'] = wrap(np.interp(tq, t, yaw))
    out.to_csv(a.out_csv, index=False)
    v = np.gradient(d[['x', 'y']].values, t, axis=0)
    print('E|v|^2 = %.2f  (mean speed %.2f m/s)' % (np.mean(np.sum(v ** 2, 1)), np.mean(np.linalg.norm(v, axis=1))))


if __name__ == '__main__':
    main()
