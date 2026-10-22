#!/usr/bin/env python3
"""Reference trajectory from GNSS only: interpolate GNSS x/y and yaw at every PLY timestamp."""
import argparse
import os

import numpy as np
import pandas as pd


def ply_timestamps(ply_dir):
    return np.array(sorted(int(f.split('_ts_')[1][:-4]) for f in os.listdir(ply_dir) if f.endswith('.ply')))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('track_dir')
    parser.add_argument('output_csv')
    args = parser.parse_args()
    ts = ply_timestamps(os.path.join(args.track_dir, 'raw_ply'))
    g = pd.read_csv(os.path.join(args.track_dir, 'sensors', 'gnss.csv'))
    yaw = np.unwrap(2 * np.arctan2(g.qz, g.qw))
    t = g.t.values.astype(np.float64)
    x = np.interp(ts, t, g.x)
    y = np.interp(ts, t, g.y)
    yw = np.interp(ts, t, yaw)
    yw = (yw + np.pi) % (2 * np.pi) - np.pi
    pd.DataFrame({'idx': np.arange(len(ts)), 'x': x, 'y': y, 'yaw': yw}).to_csv(args.output_csv, index=False)


if __name__ == '__main__':
    main()
