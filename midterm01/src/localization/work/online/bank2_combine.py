#!/usr/bin/env python3
"""Start bank over heading x forward offsets (validation): per-frame posterior weights from each hypothesis' running map
log-likelihood ('ll' of its debug csv) plus Gaussian priors on the offsets (heading sd 2 deg as tools/hyp_select.py,
forward sd 1 m = the initial position sd); output frames 0..NH-1 = posterior mean pose (circular mean heading), frames
NH.. = the full run of the MAP hypothesis at NH. Each frame uses likelihoods up to that frame only (causal).
usage: bank2_combine.py select <dir> <NH> "<heading offs>" "<fwd offs>"   -> prints the MAP pair at NH
       bank2_combine.py combine <dir> <NH> "<heading offs>" "<fwd offs>" <o> <f>  -> <dir>/T3_mean.csv, T3_map.csv"""
import sys
import numpy as np
import pandas as pd
mode, W, NH = sys.argv[1], sys.argv[2], int(sys.argv[3])
HO = sys.argv[4].split()
FO = sys.argv[5].split()
pairs = [(o, f) for o in HO for f in FO]
S = np.array([pd.read_csv(f'{W}/hyp_{o}_{f}_dbg.csv')['ll'].values[:NH].astype(float) for o, f in pairs])
prior = np.array([-0.5 * (float(o) / 2.0) ** 2 - 0.5 * (float(f) / 1.0) ** 2 for o, f in pairs])
post = S + prior[:, None]
best = int(np.argmax(post[:, NH - 1]))
if mode == 'select':
    print(*pairs[best])
    sys.exit()
P = np.array([pd.read_csv(f'{W}/hyp_{o}_{f}.csv')[['x', 'y', 'yaw']].values[:NH] for o, f in pairs])
w = np.exp(post - post.max(0))
w /= w.sum(0)
mean = np.stack([(w * P[:, :, 0]).sum(0), (w * P[:, :, 1]).sum(0),
                 np.arctan2((w * np.sin(P[:, :, 2])).sum(0), (w * np.cos(P[:, :, 2])).sum(0))], 1)
mp = P[np.argmax(post, 0), np.arange(NH)]
full = pd.read_csv(f'{W}/full.csv')
for name, head in (('T3_mean.csv', mean), ('T3_map.csv', mp)):
    out = full.copy()
    out.loc[:NH - 1, ['x', 'y', 'yaw']] = head
    out['yaw'] = (out.yaw + np.pi) % (2 * np.pi) - np.pi
    out.loc[out.yaw <= -np.pi, 'yaw'] += 2 * np.pi
    out.to_csv(f'{W}/{name}', index=False)
print('MAP at NH:', pairs[best], '| weights at frames 0/30/60/150/599 of the MAP pair:',
      ' '.join(f'{w[best, k]:.2f}' for k in (0, 30, 60, 150, NH - 1)))
