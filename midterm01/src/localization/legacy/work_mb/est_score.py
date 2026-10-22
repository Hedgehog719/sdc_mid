"""Reference-only estimate of a candidate's score from the scored files (quadratic identity, public board = all frames).

Position part: P_i^2 = P0^2 + 2 <e, d_i> + |d_i|^2 gives <e, d_i> for every scored file; the candidate's difference
from the base is projected onto their span (least squares) -> <e, d_par> exact, <e, d_perp> unknown (taken as 0).
Yaw part: same identity in yaw (deg, wrapped) with the yaw files whose 0.2 x yaw RMSE is known (yaw classes).
Prints the estimate and the in-span shares, plus forward / lateral differences to the current best by segment.
usage: est_score.py T1|T2 candidate.csv [more.csv ...]
"""
import sys

import numpy as np
import pandas as pd

S1 = 'submissions/track1/'
S2 = 'submissions/track2/'
CFG = {
    'T2': dict(
        base=('submissions/day2/track2/yaw_traj_q0.05_bev.csv', 0.28263, 'day2'),
        yaw={'day2': 0.0576, 'opt': 0.05364, 'yc9': 0.05197, 'y2': 0.04826},
        yaw_files={'day2': 'submissions/day2/track2/yaw_traj_q0.05_bev.csv', 'opt': S2 + '0.21506__t2_opt5_yopt.csv',
                   'yc9': S2 + '0.21007__t2_opt9_yc9.csv', 'y2': S2 + '0.20636__t2_opt9_yopt2.csv'},
        refs=[('0.23453__t2_rest06_pc.csv', 0.23453, 'day2'), ('0.25031__t2_gm.csv', 0.25031, 'day2'),
              ('0.23178__t2_blend.csv', 0.23178, 'day2'), ('0.23658__t2_mkS_blend_yopt.csv', 0.23658, 'opt'),
              ('0.22364__t2_t2fit_blend_c_yopt.csv', 0.22364, 'opt'), ('0.21506__t2_opt5_yopt.csv', 0.21506, 'opt'),
              ('0.25803__t2_t2fit_blend_yopt.csv', 0.25803, 'opt'), ('0.21440__t2_opt7_yopt.csv', 0.21440, 'opt'),
              ('0.23151__t2_opt7_imgc.csv', 0.23151, 'opt'), ('0.22698__t2_opt8_vertc.csv', 0.22698, 'opt'),
              ('0.21174__t2_opt9.csv', 0.21174, 'opt'), ('0.31496__t2_best_algostart.csv', 0.31496, 'y2'),
              ('0.28674__t2_best_algostart_fwd.csv', 0.28674, 'y2'), ('0.17429__t2_opt10.csv', 0.17429, 'y2')],
        best=S2 + '0.17429__t2_opt10.csv', start=330, d=S2),
    'T1': dict(
        base=(S1 + '0.61440__h1_oldstart.csv', 0.61440, 'H'),
        yaw={'H': 0.143, 'cur': 0.13411, 'cur2': 0.13314},
        yaw_files={'H': S1 + '0.61440__h1_oldstart.csv', 'cur': S1 + '0.50997__t1_v2.csv',
                   'cur2': S1 + '0.46206__t1_opt8_yc8x121.csv'},
        refs=[('0.66298__t1_a13.csv', 0.66298, 'H'), ('0.57658__t1_gm.csv', 0.57658, 'H'),
              ('0.62128__t1_h1_pc.csv', 0.62128, 'H'), ('0.54851__t1_mb.csv', 0.54851, 'H'),
              ('0.52496__t1_mk1S.csv', 0.52496, 'H'), ('0.50997__t1_v2.csv', 0.50997, 'cur'),
              ('0.46775__t1_opt6_vertc.csv', 0.46775, 'cur'), ('0.55138__t1_opt7_startlat.csv', 0.55138, 'cur'),
              ('0.46303__t1_opt8.csv', 0.46303, 'cur'), ('0.56263__t1_best_startfwd05.csv', 0.56263, 'cur2'),
              ('0.42639__t1_opt9_endlat.csv', 0.42639, 'cur2'), ('0.41668__t1_opt10.csv', 0.41668, 'cur2'),
              ('0.41664__t1_opt10_algostart.csv', 0.41664, 'cur2'), ('0.41123__t1_opt11.csv', 0.41123, 'cur2')],
        best=S1 + '0.41123__t1_opt11.csv', start=330, d=S1),
}
T = sys.argv[1]
c = CFG[T]
base = pd.read_csv(c['base'][0])
n = len(base)
Y = c['yaw']
vec = lambda df: np.concatenate([df.x.values - base.x.values, df.y.values - base.y.values])  # noqa: E731
refs = [(c['d'] + f, s, k) for f, s, k in c['refs']]
V = np.array([vec(pd.read_csv(f)) for f, _, _ in refs])
P0 = c['base'][1] - Y[c['base'][2]]
P = np.array([s - Y[k] for _, s, k in refs])
cc = (P ** 2 - P0 ** 2 - np.einsum('ij,ij->i', V, V) / n) / 2
# yaw: classes with a known yaw file
ycl = list(c['yaw_files'])
ybase = c['base'][2]
yb = pd.read_csv(c['yaw_files'][ybase]).yaw.values
wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi  # noqa: E731
R0 = Y[ybase] / 0.2
YV = np.array([np.degrees(wrap(pd.read_csv(c['yaw_files'][k]).yaw.values - yb)) for k in ycl if k != ybase])
YR = np.array([Y[k] / 0.2 for k in ycl if k != ybase])
yc = (YR ** 2 - R0 ** 2 - np.einsum('ij,ij->i', YV, YV) / n) / 2
B = pd.read_csv(c['best'])
for f in sys.argv[2:]:
    D = pd.read_csv(f)
    v = vec(D)
    w = np.linalg.lstsq(V.T, v, rcond=None)[0]
    vp = V.T @ w
    share = (vp @ vp) / max(v @ v, 1e-12)
    P2 = P0 ** 2 + 2 * w @ cc + v @ v / n
    dy = np.degrees(wrap(D.yaw.values - yb))
    wy = np.linalg.lstsq(YV.T, dy, rcond=None)[0]
    dyp = YV.T @ wy
    ysh = (dyp @ dyp) / max(dy @ dy, 1e-12)
    R2 = R0 ** 2 + 2 * wy @ yc + dy @ dy / n
    Pe, Re = np.sqrt(max(P2, 0)), np.sqrt(max(R2, 0))
    print(f'{f}: estimated score {Pe + 0.2 * Re:.4f} = position {Pe:.4f} (in span {share * 100:.0f} %) '
          f'+ 0.2 x yaw {Re:.3f} deg (in span {ysh * 100:.0f} %)   [current best {c["best"].split("/")[-1].split("__")[0]}]')
    yw = B.yaw.values
    dx, dyy = D.x.values - B.x.values, D.y.values - B.y.values
    fw, lt = np.cos(yw) * dx + np.sin(yw) * dyy, -np.sin(yw) * dx + np.cos(yw) * dyy
    da = np.degrees(wrap(D.yaw.values - yw))
    for nm, sl in (('all', slice(None)), (f'start 0-{c["start"]}', slice(0, c['start'])), (f'after {c["start"]}', slice(c['start'], None))):
        print(f'   vs best, {nm:12s}: forward mean {fw[sl].mean():+.3f} rms {np.sqrt(np.mean(fw[sl] ** 2)):.3f} | lateral mean '
              f'{lt[sl].mean():+.3f} rms {np.sqrt(np.mean(lt[sl] ** 2)):.3f} | yaw mean {da[sl].mean():+.3f} rms {np.sqrt(np.mean(da[sl] ** 2)):.3f} deg')
