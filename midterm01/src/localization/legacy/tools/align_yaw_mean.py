#!/usr/bin/env python3
"""Shift a candidate's yaw so that its mean equals the reference's (the probed bias-free yaw level).

Usage: align_yaw_mean.py candidate.csv reference.csv [out.csv]   (in place when out is omitted)
"""
import sys

import numpy as np
import pandas as pd

c = pd.read_csv(sys.argv[1])
r = pd.read_csv(sys.argv[2])
d = np.angle(np.exp(1j * (c.yaw.values - r.yaw.values)))
off = float(np.mean(d))
c['yaw'] = c.yaw.values - off  # keep the file's angle representation
c.to_csv(sys.argv[3] if len(sys.argv) > 3 else sys.argv[1], index=False)
print(f'yaw mean offset {np.degrees(off):+.3f} deg removed')
