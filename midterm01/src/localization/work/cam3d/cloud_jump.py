"""Relative yaw between camera clouds k and k+step (2D ICP on their own points) vs the IMU yaw change: a mismatch
means the camera cloud's predicted frame (MapAnything, no calibrated extrinsics) rotated, not the vehicle."""
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
    r = np.hypot(Q[:, 0], Q[:, 1]) * s
    Q = Q[(r > 3) & (r < 25)] * s
    hz = Q[:, 2] - (pa * Q[:, 0] + pb * Q[:, 1] + pc) * s
    P = voxel(Q[hz > 0.3][:, :2], 0.15)            # raised structure, top view
    return P, s, np.degrees(np.arctan2(pb, 1.0)), np.degrees(np.arctan(pa))
def icp2d(A, B, th0, t0):
    tree = cKDTree(B); th, t = th0, t0.copy()
    for it in range(25):
        c, s = np.cos(th), np.sin(th); W = A @ np.array([[c, s], [-s, c]]) + t
        d, j = tree.query(W, distance_upper_bound=0.8 if it < 10 else 0.3)
        ok = np.isfinite(d)
        if ok.sum() < 50: return None
        a, b = A[ok], B[j[ok]]; ma, mb = a.mean(0), b.mean(0)
        H = (a - ma).T @ (b - mb); U, _, Vt = np.linalg.svd(H); Rm = (U @ Vt).T
        if np.linalg.det(Rm) < 0: Vt[1] *= -1; Rm = (U @ Vt).T
        th = np.arctan2(Rm[1, 0], Rm[0, 0]); t = mb - ma @ Rm.T
    return th, t, ok.mean()
for a in [int(v) for v in sys.argv[1].split(',')]:
    print(f'== around {a}: frame k -> k+{step}: cloud-relative yaw vs IMU (deg); cloud ground roll/pitch; scale')
    for k in range(a - 6, a + 7, step):
        A, sa, ra, pa_ = cloud(k); Bc, sb, rb, pb_ = cloud(k + step)
        dyaw_imu = iy[k + step] - iy[k]
        dist = np.hypot(V.x[k + step] - V.x[k], V.y[k + step] - V.y[k])
        # points of cloud k+step expressed in frame k: p_k = R(dyaw) p_{k+3} + (dist, 0)
        res = icp2d(Bc, A, dyaw_imu, np.array([dist, 0.0]))
        if res is None: print(f'   {k}: ICP failed'); continue
        th, t, inl = res
        print(f'   {k:4d}->{k+step:4d}: cloud {np.degrees(th):+.2f} IMU {np.degrees(dyaw_imu):+.2f} -> diff {np.degrees(th - dyaw_imu):+.2f} | shift {t[0]:+.2f},{t[1]:+.2f} (odo {dist:.2f}) inl {inl:.2f} | scale {sa:.3f}->{sb:.3f} roll {ra:+.2f}->{rb:+.2f} pitch {pa_:+.2f}->{pb_:+.2f}')
