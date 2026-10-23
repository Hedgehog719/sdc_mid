#!/usr/bin/env python3
"""Causal selection in a bank of heading hypotheses (filters started with different frame-0 headings).

The frame-0 lane match can be flat over several degrees (Track03: peaks 0.19-0.22 from +13.5 to +21.5 deg), and a
filter started a few degrees off may never lock. localize_online.py is therefore started with several heading offsets
(--init_offset) for the first N frames. For every frame k the output is the pose of the hypothesis with the highest
posterior up to frame k: the log-likelihood of its map measurements so far (Gaussian term of every accepted lane / near
measurement, a gate-sized penalty for every one outside the gate; column 'll' of the debug csv) plus the log of a
Gaussian prior (sd PRIOR_SD) on the heading offset, centred on the frame-0 search, the best single-frame estimate
(an exact tie goes to the smallest heading offset). Plain maximum likelihood (flat prior) took the +3 deg hypothesis
on a near-tie at frame 6 (log-likelihoods -1.5 / -1.6 / -2.0 for +3 / -1.5 / 0 after two measurements); a margin of 5
that preferred the 0 offset kept a worse hypothesis for 25 frames on Track03. At frame N the best hypothesis is kept
(the others are pruned) and its full-length run supplies the frames after N. Every output uses data up to its own
frame only.

usage: hyp_select.py select <N> <offsets a,b,..> <hyp.csv:hyp_debug.csv> ...   -> prints the winning index at frame N
       hyp_select.py combine <out.csv> <N> <winner_full.csv> <offsets a,b,..> <hyp.csv:hyp_debug.csv> ...
"""
import sys

import numpy as np
import pandas as pd


def scores(dbg_files, n):
    return np.array([pd.read_csv(f)['ll'].values[:n].astype(float) for f in dbg_files])   # (hypotheses, frames)


MARGIN = 0.0
PRIOR_SD = 2.0   # deg: the frame-0 lane-match heading peak is flat over a few degrees


def choice(S, offsets):
    post = S - 0.5 * (np.asarray(offsets, float)[:, None] / PRIOR_SD) ** 2
    best = post.max(0)
    order = np.argsort(np.abs(offsets), kind='stable')
    return np.array([int(next(h for h in order if post[h, k] >= best[k] - MARGIN)) for k in range(S.shape[1])])


if __name__ == '__main__':
    mode = sys.argv[1]
    if mode == 'select':
        n = int(sys.argv[2])
        offs = np.array([float(v) for v in sys.argv[3].split(',')])
        pairs = [p.split(':') for p in sys.argv[4:]]
        S = scores([d for _, d in pairs], n)
        ch = choice(S, offs)
        print(int(ch[n - 1]))
        sys.stderr.write('scores at frame %d: %s\n' % (n, ' '.join(f'{v:.0f}' for v in S[:, n - 1])))
    else:
        out, n, full = sys.argv[2], int(sys.argv[3]), sys.argv[4]
        offs = np.array([float(v) for v in sys.argv[5].split(',')])
        pairs = [p.split(':') for p in sys.argv[6:]]
        S = scores([d for _, d in pairs], n)
        ch = choice(S, offs)
        H = [pd.read_csv(c) for c, _ in pairs]
        F = pd.read_csv(full)
        R = F.copy()
        for k in range(n):
            R.iloc[k, 1:] = H[ch[k]].iloc[k, 1:].values
        R.to_csv(out, index=False)
        sw = int(np.sum(ch[1:] != ch[:-1]))
        sys.stderr.write(f'frames 0-{n - 1}: hypothesis per frame (switches {sw}), final {ch[n - 1]}; frames {n}+: full run -> {out}\n')
