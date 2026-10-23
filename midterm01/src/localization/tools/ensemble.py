#!/usr/bin/env python3
"""Frame-wise mean of several causal localization runs of the same track (positions averaged, headings as circular
mean, output in (-pi, pi]). Still causal: the pose of frame k averages the members' poses of frame k, each of which
uses frames <= k only. Used for Track01, whose filter is chaotic in sparse-marking sections (near-identical settings
differ by +-0.02 in the score): the members differ in the two uncertain heading settings (initial heading sd x lane
heading sigma) and the mean averages their heading excursions out.
usage: ensemble.py <out.csv> <member.csv> [<member.csv> ...]"""
import sys

import numpy as np
import pandas as pd

out, runs = sys.argv[1], sys.argv[2:]
T = [pd.read_csv(p) for p in runs]
n = min(len(t) for t in T)
assert all(len(t) == n for t in T), 'members differ in length'
x = np.mean([t.x.values for t in T], 0)
y = np.mean([t.y.values for t in T], 0)
yaw = np.arctan2(np.mean([np.sin(t.yaw.values) for t in T], 0), np.mean([np.cos(t.yaw.values) for t in T], 0))
pd.DataFrame({'idx': T[0].idx.values, 'x': x, 'y': y, 'yaw': yaw}).to_csv(out, index=False)
print(f'ensemble of {len(T)} runs -> {out}')
