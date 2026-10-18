#!/usr/bin/env python3
"""Apply exported pole review decisions: keep 'keep' (and optionally 'undecided') poles -> poles_curated.npy.

usage: apply_pole_review.py /root/data/Track1 pole_review_Track1.json [--drop_undecided]
"""
import argparse
import json
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('review_json')
ap.add_argument('--drop_undecided', action='store_true', help='keep only poles explicitly marked Keep')
a = ap.parse_args()
P = np.load(os.path.join(a.track_dir, 'poles.npy'))
r = json.load(open(a.review_json))
keep = set(r['keep']) | (set() if a.drop_undecided else set(r.get('undecided', [])))
idx = np.array(sorted(keep), int)
np.save(os.path.join(a.track_dir, 'poles_curated.npy'), P[idx])
print(f"{r['track']}: {len(idx)} of {len(P)} poles kept ({len(r['reject'])} rejected) -> poles_curated.npy")
