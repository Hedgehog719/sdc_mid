"""T2 start: lane-fix positions vs the GNSS-based prior and vs the board-derived start (opt10), per segment.
usage: start_fix_cmp.py fixes.npz [more.npz ...]"""
import sys

import numpy as np
import pandas as pd

ref = pd.read_csv('submissions/track2/0.17429__t2_opt10.csv')     # board-derived start (check only)
gns = pd.read_csv('submissions/track2/0.20636__t2_opt9_yopt2.csv')  # start = raw GNSS (before the start probes)
segs = ((0, 250), (250, 330), (330, 450))
for f in sys.argv[1:]:
    z = dict(np.load(f))
    i = z['idx'].astype(int)
    ok = i < 450
    if 'max_yaw' in z:
        ok &= np.abs(z['dyaw']) < z['max_yaw'] - 0.05          # drop fixes at the yaw search limit
    out = [f'{f.split("/")[-1]:24s} n {ok.sum():3d} peak median {np.median(z["peak"][ok]):.3f}']
    for nm, T in (('vs GNSS start', gns), ('vs board start', ref)):
        yw = T.yaw.values[i]
        dx, dy = z['mx'] - T.x.values[i], z['my'] - T.y.values[i]
        fw, lt = np.cos(yw) * dx + np.sin(yw) * dy, -np.sin(yw) * dx + np.cos(yw) * dy
        parts = []
        for a, b in segs:
            m = ok & (i >= a) & (i < b)
            if m.sum():
                parts.append(f'{a}-{b}: fwd {np.median(fw[m]):+.2f} lat {np.median(lt[m]):+.2f}')
        out.append(f'  {nm}: ' + ' | '.join(parts))
    print('\n'.join(out))
