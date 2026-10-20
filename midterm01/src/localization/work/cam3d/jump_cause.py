"""Where do the camera-cloud rotations come from? Per frame: relative yaw to the cloud 3 frames later from the FRONT
sector (|azimuth| < 30 deg, front camera only), the LEFT and the RIGHT sectors (side cameras), each vs the IMU;
plus scale / ground roll / pitch changes and point counts."""
import sys, numpy as np, pandas as pd
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
sys.path.insert(0, 'work/cam3d')
from icp import read_ply, voxel
T = 'Track2'; step = 3
idx = pd.read_csv(f'/root/data/{T}/3cameras_ply/index.csv'); ft = idx.f_ts.values.astype(np.float64)
im = pd.read_csv(f'/root/data/{T}/sensors/imu.csv')
iy = np.interp(ft, im.stamp.values.astype(np.float64), np.unwrap(Rotation.from_quat(im[['qx','qy','qz','qw']].values).as_euler('ZYX')[:, 0]))
V = pd.read_csv('submissions/day7/track2/t2_pipe_v5.csv')
def cloud(k):
    Q = read_ply(f'/root/data/{T}/3cameras_ply/' + idx.ply[k])
    near = (Q[:, 0] > 3) & (Q[:, 0] < 15) & (np.abs(Q[:, 1]) < 3)
    G = Q[near]; z0 = np.percentile(G[:, 2], 20); gg = G[np.abs(G[:, 2] - z0) < 0.25]
    pa, pb, pc = np.linalg.lstsq(np.c_[gg[:, 0], gg[:, 1], np.ones(len(gg))], gg[:, 2], rcond=None)[0]
    s = 1.70 / max(-pc, 0.5)
    hz = (Q[:, 2] - (pa * Q[:, 0] + pb * Q[:, 1] + pc)) * s
    r = np.hypot(Q[:, 0], Q[:, 1]) * s
    az = np.degrees(np.arctan2(Q[:, 1], Q[:, 0]))
    m = (r > 3) & (r < 25) & (hz > 0.3) & (hz < 6)
    P = np.c_[Q[m, :2] * s, az[m]]
    return P, s, np.degrees(np.arctan(pb)), np.degrees(np.arctan(pa)), len(Q)
def icp(A, B, th0, t0):
    if len(A) < 80 or len(B) < 80: return np.nan
    tree = cKDTree(B); th, t = th0, t0.copy()
    for it in range(20):
        c, s = np.cos(th), np.sin(th); W = A @ np.array([[c, s], [-s, c]]) + t
        d, j = tree.query(W, distance_upper_bound=0.8 if it < 8 else 0.3); ok = np.isfinite(d)
        if ok.sum() < 40: return np.nan
        p, q = A[ok], B[j[ok]]; mp, mq = p.mean(0), q.mean(0)
        U, _, Vt = np.linalg.svd((p - mp).T @ (q - mq)); Rm = (U @ Vt).T
        if np.linalg.det(Rm) < 0: Vt[1] *= -1; Rm = (U @ Vt).T
        th = np.arctan2(Rm[1, 0], Rm[0, 0]); t = mq - mp @ Rm.T
    return th
rows = []
lo, hi = int(sys.argv[1]), int(sys.argv[2])
for k in range(lo, hi, step):
    A, sa, ra, pa_, na = cloud(k); B, sb, rb, pb_, nb = cloud(k + step)
    dimu = iy[k + step] - iy[k]; dist = np.hypot(V.x[k + step] - V.x[k], V.y[k + step] - V.y[k])
    out = {}
    for nm, sel in (('all', lambda P: np.ones(len(P), bool)), ('front', lambda P: np.abs(P[:, 2]) < 30),
                    ('left', lambda P: (P[:, 2] > 30) & (P[:, 2] < 120)), ('right', lambda P: (P[:, 2] < -30) & (P[:, 2] > -120))):
        th = icp(voxel(B[sel(B)][:, :2], 0.15), voxel(A[sel(A)][:, :2], 0.15), dimu, np.array([dist, 0.0]))
        out[nm] = np.degrees(th - dimu)
    rows.append(dict(k=k, **{f'e_{n}': v for n, v in out.items()}, ds=(sb - sa) / sa * 100, droll=rb - ra, dpitch=pb_ - pa_,
                     npts=na, yawrate=np.degrees(dimu) / (step / 30.0), v=dist / (step / 30.0)))
R = pd.DataFrame(rows)
pd.set_option('display.width', 200)
print(R.round(2).to_string(index=False, max_rows=60))
print('\nrobust spread (1.4826 MAD) of the yaw disagreement with the IMU, deg:')
for c in ('e_all', 'e_front', 'e_left', 'e_right'):
    v = R[c].dropna(); print(f'  {c:8s} n {len(v):3d} spread {1.4826 * np.median(np.abs(v - np.median(v))):.2f} median {np.median(v):+.2f}')
print('correlation of |e_all| with: ' + ' '.join(f'{c} {np.corrcoef(np.abs(R.e_all.fillna(0)), np.abs(R[c]))[0,1]:+.2f}' for c in ('ds', 'droll', 'dpitch', 'yawrate')))
print('correlation of e_left with e_right: %.2f, e_front with e_left: %.2f' % (R[['e_left', 'e_right']].dropna().corr().iloc[0, 1], R[['e_front', 'e_left']].dropna().corr().iloc[0, 1]))
