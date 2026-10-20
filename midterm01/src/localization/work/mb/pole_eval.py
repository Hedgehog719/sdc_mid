"""Local check of pole fixes: compare with forward-sharp lane fixes (|frame gap| <= 3, sharp_f >= thr) after removing
the trajectory motion between the two frames; prints bias and robust spread of the difference (no leaderboard)."""
import sys, numpy as np, pandas as pd
lane = dict(np.load(sys.argv[1])); T = pd.read_csv(sys.argv[2]); thr = float(sys.argv[3])
li = lane['idx'].astype(int); keep = (lane['sharp_f'] >= thr) & (np.abs(lane['dyaw']) < lane['max_yaw'] - 0.05)
li, lx, ly = li[keep], lane['mx'][keep], lane['my'][keep]
x, y, yw = T.x.values, T.y.values, T.yaw.values
for f in sys.argv[4:]:
    z = dict(np.load(f)); pi = z['idx'].astype(int)
    rows = []
    for k, i in enumerate(pi):
        j = np.argmin(np.abs(li - i))
        if abs(li[j] - i) > 3: continue
        ex = (z['mx'][k] - x[i]) - (lx[j] - x[li[j]]); ey = (z['my'][k] - y[i]) - (ly[j] - y[li[j]])
        c, s = np.cos(yw[i]), np.sin(yw[i])
        rows.append((i, c * ex + s * ey, -s * ex + c * ey))
    if not rows: print(f, 'no pairs'); continue
    R = np.array(rows); rs = lambda v: 1.4826 * np.median(np.abs(v - np.median(v)))
    for nm, m in (('all', np.ones(len(R), bool)), ('start <330', R[:, 0] < 330), ('>=330', R[:, 0] >= 330)):
        if m.sum() < 3: continue
        print(f'{f.split("/")[-1]:18s} {nm:10s} pairs {m.sum():3d} | pole - lane fwd median {np.median(R[m,1]):+.2f} robust sd {rs(R[m,1]):.2f} | lat median {np.median(R[m,2]):+.2f} robust sd {rs(R[m,2]):.2f} | |fwd|>0.5: {np.mean(np.abs(R[m,1]-np.median(R[m,1]))>0.5)*100:.0f}%')
