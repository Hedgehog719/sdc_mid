#!/usr/bin/env python3
"""Shift a candidate's yaw so its mean equals a reference submission whose mean yaw bias was removed by probing."""
import sys

import numpy as np
import pandas as pd

cand, ref = pd.read_csv(sys.argv[1]), pd.read_csv(sys.argv[2])
d = np.mean((cand.yaw - ref.yaw + np.pi) % (2 * np.pi) - np.pi)
cand['yaw'] = (cand.yaw - d + np.pi) % (2 * np.pi) - np.pi
cand.to_csv(sys.argv[1], index=False)
print(f'{sys.argv[1]}: yaw mean shifted by {np.degrees(-d):+.3f} deg')
