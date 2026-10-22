#!/usr/bin/env python3
"""GNSS-only trajectory variants used to probe the error budget (time lag, yaw source, smoothing)."""
import argparse
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(__file__))
from common import Sensors, ply_frames, wrap  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('track_dir')
    parser.add_argument('output_csv')
    parser.add_argument('--lag', type=float, default=0.0, help='seconds added to frame time before GNSS lookup')
    parser.add_argument('--yaw', choices=['gnss', 'imu', 'motion'], default='gnss')
    args = parser.parse_args()
    ts, _ = ply_frames(os.path.join(args.track_dir, 'raw_ply'))
    s = Sensors(args.track_dir)
    tq = ts + args.lag * 1e9
    xyz = s.gnss_xyz(tq)
    if args.yaw == 'gnss':
        yaw = s.gnss_yaw(tq)
    elif args.yaw == 'imu':
        imu_yaw_at_gnss = np.unwrap(s.imu_rot(s.gt).as_euler('ZYX')[:, 0])
        offset = np.median(wrap(s.gyaw - imu_yaw_at_gnss))
        yaw = np.unwrap(s.imu_rot(tq).as_euler('ZYX')[:, 0]) + offset
    else:
        d = np.gradient(xyz[:, :2], axis=0)
        yaw = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    pd.DataFrame({'idx': np.arange(len(ts)), 'x': xyz[:, 0], 'y': xyz[:, 1], 'yaw': wrap(yaw)}).to_csv(args.output_csv, index=False)


if __name__ == '__main__':
    main()
