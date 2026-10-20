import numpy as np, pandas as pd, sys
sys.path.insert(0, 'work/cam3d')
from icp import *
from scipy.spatial import cKDTree
T = sys.argv[1]; frames = [int(v) for v in sys.argv[2].split(',')]
best = {'Track2': 'submissions/track2/0.17429__t2_opt10.csv', 'Track1': 'submissions/track1/0.41123__t1_opt11.csv'}[T]
idx = pd.read_csv(f'/root/data/{T}/3cameras_ply/index.csv'); tr = pd.read_csv(best)
M = np.load(f'/root/data/{T}/map_clean.npy', mmap_mode='r')
H = 1.70
for fr in frames:
    Q = read_ply(f'/root/data/{T}/3cameras_ply/' + idx.ply[fr])
    r = np.hypot(Q[:, 0], Q[:, 1]); Q = voxel(Q[(r > 3) & (r < 30)], 0.25)
    near = (Q[:, 0] > 3) & (Q[:, 0] < 12) & (np.abs(Q[:, 1]) < 2.5)
    gz = np.percentile(Q[near, 2], 20) if near.sum() > 50 else -1.9
    s0 = H / max(-gz, 0.5)
    x, y, yaw = tr.x[fr], tr.y[fr], tr.yaw[fr]
    m = (np.abs(M[:, 0] - x) < 40) & (np.abs(M[:, 1] - y) < 40); P = voxel(np.asarray(M[m], dtype=np.float64), 0.1)
    tree = cKDTree(P)
    gmap = np.percentile(P[np.hypot(P[:, 0] - x, P[:, 1] - y) < 4, 2], 10)
    c, s = np.cos(yaw), np.sin(yaw); R0 = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    for (pf, pl) in ((0, 0), (0.5, 0), (-0.5, 0), (0, 0.3), (0, -0.3)):
     t0 = np.array([x + c * pf - s * pl, y + s * pf + c * pl, gmap + H])
     res = register(Q, tree, R0, t0, s0)
     if res is None: print(fr, 'fail'); continue
     sc, R, t, inl, med = res
     dx, dy = t[0] - x, t[1] - y
     fw, lt = c * dx + s * dy, -s * dx + c * dy
     ya = np.degrees(np.arctan2(R[1, 0], R[0, 0]) - yaw); ya = (ya + 180) % 360 - 180
     print(f'init ({pf:+.1f},{pl:+.1f}) {T} f{fr}: s0 {s0:.3f} -> scale {sc:.3f}, inliers<0.2 {inl*100:.0f}%, med {med:.3f} | cam vs best-vehicle fwd {fw:+.2f} lat {lt:+.2f} dz {t[2]-gmap:+.2f} dyaw {ya:+.2f}')
