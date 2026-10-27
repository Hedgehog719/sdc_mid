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

def combine_runs(frames):
	if not frames or not len(frames[0]):
		raise ValueError('at least one nonempty causal run is required')
	expected = np.arange(len(frames[0]))
	for frame in frames:
		if list(frame.columns) != ['idx', 'x', 'y', 'yaw'] or len(frame) != len(expected):
			raise ValueError('members differ in length or schema')
		if not np.array_equal(frame.idx.to_numpy(), expected) or not np.isfinite(frame.to_numpy()).all():
			raise ValueError('members must have identical consecutive indices and finite poses')
	position_x = np.mean([frame.x.values for frame in frames], 0)
	position_y = np.mean([frame.y.values for frame in frames], 0)
	heading = np.arctan2(np.mean([np.sin(frame.yaw.values) for frame in frames], 0),
						 np.mean([np.cos(frame.yaw.values) for frame in frames], 0))
	return pd.DataFrame({'idx': expected, 'x': position_x, 'y': position_y, 'yaw': heading})


if __name__ == '__main__':
	output, runs = sys.argv[1], sys.argv[2:]
	result = combine_runs([pd.read_csv(path) for path in runs])
	result.to_csv(output, index=False)
	print(f'ensemble of {len(runs)} runs -> {output}')
