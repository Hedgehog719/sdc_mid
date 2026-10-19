#!/usr/bin/env python3
"""GNSS-free local standard: agreement between per-camera fixes of the same frames.

img_match run with a single camera gives independent fixes per camera (shared map and pose prior only). Their
pairwise differences (in the vehicle frame: forward / lateral / yaw) measure fix noise plus calibration
inconsistency between cameras - no ground truth or GNSS involved. Lower = better calibration / matcher.
usage: eval_cams.py <tag_f.npz> <tag_fl.npz> <tag_fr.npz> [--label NAME]
"""
import argparse
import itertools

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('files', nargs=3)
ap.add_argument('--label', default='')
a = ap.parse_args()
Z = [dict(np.load(f)) for f in a.files]
names = ['f', 'fl', 'fr']
common = sorted(set.intersection(*[set(z['idx'].astype(int)) for z in Z]))


def get(z, key):
    k = np.searchsorted(z['idx'].astype(int), common)
    return z[key][k]


def rob(v):
    med = np.median(v)
    mad = 1.4826 * np.median(np.abs(v - med)) + 1e-9
    k = np.abs(v - med) < 3 * mad
    return np.sqrt(np.mean(v[k] ** 2)), med, k.mean()


out = f'{a.label:18s} n {len(common):3d} |'
for (i, j) in itertools.combinations(range(3), 2):
    dl = get(Z[i], 'lat') - get(Z[j], 'lat')
    df = get(Z[i], 'fwd') - get(Z[j], 'fwd')
    dw = np.degrees(get(Z[i], 'dyaw') - get(Z[j], 'dyaw'))
    lr, lm, _ = rob(dl)
    fr, fm, _ = rob(df)
    wr, wm, _ = rob(dw)
    out += f' {names[i]}-{names[j]}: lat {lr:.3f} ({lm:+.2f}) fwd {fr:.3f} ({fm:+.2f}) yaw {wr:.2f} ({wm:+.2f}) |'
print(out)
