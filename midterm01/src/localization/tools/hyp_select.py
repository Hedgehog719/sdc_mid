#!/usr/bin/env python3
"""Causal selection in a bank of heading hypotheses (filters started with different frame-0 headings).

The frame-0 lane match can be flat over several degrees (Track03: peaks 0.19-0.22 from +13.5 to +21.5 deg), and a
filter started a few degrees off may never lock. localize_online.py is therefore started with several heading offsets
(--init_offset) for the first N frames. For every frame k the output is the pose of the hypothesis with the highest
score up to frame k (score = accepted map measurements so far: lane forward + lane lateral + 2 x lane heading + near
lateral), ties going to the first hypothesis listed. At frame N the best hypothesis is kept (the others are pruned)
and its full-length run supplies the frames after N. Every output uses data up to its own frame only.

usage: hyp_select.py select <N> <hyp.csv:hyp_debug.csv> ...        -> prints the winning hypothesis index at frame N
       hyp_select.py combine <out.csv> <N> <winner_full.csv> <hyp.csv:hyp_debug.csv> ...
"""
import sys

import numpy as np
import pandas as pd


def scores(dbg_files, n):
    S = []
    for f in dbg_files:
        D = pd.read_csv(f)
        acc = np.zeros(n)
        for c, w in (('ln_fa', 1.0), ('ln_la', 1.0), ('ln_ya', 2.0), ('nr_la', 1.0)):
            if c in D:
                v = np.nan_to_num(D[c].values[:n].astype(float))
                acc[:len(v)] += w * v
        S.append(np.cumsum(acc))
    return np.array(S)                      # (hypotheses, frames)


def choice(S):
    best = S.max(0)
    return np.array([int(np.flatnonzero(S[:, k] == best[k])[0]) for k in range(S.shape[1])])


if __name__ == '__main__':
    mode = sys.argv[1]
    if mode == 'select':
        n = int(sys.argv[2])
        pairs = [p.split(':') for p in sys.argv[3:]]
        S = scores([d for _, d in pairs], n)
        ch = choice(S)
        print(int(ch[n - 1]))
        sys.stderr.write('scores at frame %d: %s\n' % (n, ' '.join(f'{v:.0f}' for v in S[:, n - 1])))
    else:
        out, n, full = sys.argv[2], int(sys.argv[3]), sys.argv[4]
        pairs = [p.split(':') for p in sys.argv[5:]]
        S = scores([d for _, d in pairs], n)
        ch = choice(S)
        H = [pd.read_csv(c) for c, _ in pairs]
        F = pd.read_csv(full)
        R = F.copy()
        for k in range(n):
            R.iloc[k, 1:] = H[ch[k]].iloc[k, 1:].values
        R.to_csv(out, index=False)
        sw = int(np.sum(ch[1:] != ch[:-1]))
        sys.stderr.write(f'frames 0-{n - 1}: hypothesis per frame (switches {sw}), final {ch[n - 1]}; frames {n}+: full run -> {out}\n')
