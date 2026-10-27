#!/usr/bin/env python3
"""Fill the VO pairs that the full-block matcher gave up on (resp 0: lead vehicle close ahead) with the pairs of a
masked-matching run (bev_vo.py --mask_tol), keep every other pair unchanged. Both inputs are causal per pair.
usage: vo_fill.py <vo.npz> <vo_masked.npz> <out.npz>"""
import sys
import numpy as np
a, b = np.load(sys.argv[1]), np.load(sys.argv[2])
assert (a['j'] == b['j']).all() and (a['k'] == b['k']).all()
fill = (a['resp'] <= 0) & (b['resp'] > 0)
out = {k: a[k].copy() for k in a.files}
for k in ('fwd', 'lat', 'resp', 'speed'):
    out[k][fill] = b[k][fill]
np.savez(sys.argv[3], **out)
print(f'filled {int(fill.sum())} of {len(fill)} pairs -> {sys.argv[3]}')
