"""Is the scale jitter the model's or ours? Relative scale of raw clouds k -> k+3 from 2D similarity ICP on raised
structure (no rescaling) vs the ratio of our ground-based scales (camera 1.70 m above the cloud's ground plane)."""
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
def raw(k):
    Q = read_ply(f'/root/data/{T}/3cameras_ply/' + idx.ply[k])
    near = (Q[:, 0] > 3) & (Q[:, 0] < 15) & (np.abs(Q[:, 1]) < 3)
    G = Q[near]; z0 = np.percentile(G[:, 2], 20); gg = G[np.abs(G[:, 2] - z0) < 0.25]
    pa, pb, pc = np.linalg.lstsq(np.c_[gg[:, 0], gg[:, 1], np.ones(len(gg))], gg[:, 2], rcond=None)[0]
    hz = Q[:, 2] - (pa * Q[:, 0] + pb * Q[:, 1] + pc)
    r = np.hypot(Q[:, 0], Q[:, 1])
    m = (r > 3) & (r < 30) & (hz > 0.3) & (hz < 6)
    return voxel(Q[m, :2], 0.15), 1.70 / max(-pc, 0.5), -pc, np.median(Q[near][:, 2])
def sim_icp(A, B, t0, th0):
    tree = cKDTree(B); s, th, t = 1.0, th0, t0.copy()
    for it in range(25):
        c, sn = np.cos(th), np.sin(th); W = s * A @ np.array([[c, sn], [-sn, c]]) + t
        d, j = tree.query(W, distance_upper_bound=1.0 if it < 8 else 0.35); ok = np.isfinite(d)
        if ok.sum() < 60: return np.nan, np.nan
        p, q = A[ok], B[j[ok]]; mp, mq = p.mean(0), q.mean(0); p0, q0 = p - mp, q - mq
        U, S, Vt = np.linalg.svd(p0.T @ q0); Rm = (U @ Vt).T
        if np.linalg.det(Rm) < 0: Vt[1] *= -1; Rm = (U @ Vt).T
        th = np.arctan2(Rm[1, 0], Rm[0, 0]); s = S.sum() / (p0 ** 2).sum(); t = mq - s * mp @ Rm.T
    return s, th
rows = []
for k in range(int(sys.argv[1]), int(sys.argv[2]), step):
    A, sa, ha, za = raw(k); B, sb, hb, zb = raw(k + step)
    dist = np.hypot(V.x[k + step] - V.x[k], V.y[k + step] - V.y[k])
    # translation guess in raw units: metric distance / scale of the target cloud
    s_icp, th = sim_icp(B, A, np.array([dist / sa, 0.0]), iy[k + step] - iy[k])
    rows.append(dict(k=k, icp_rel_scale=s_icp, ground_ratio=sb / sa, raw_cam_height_k=ha, dyaw_err=np.degrees(th - (iy[k + step] - iy[k]))))
R = pd.DataFrame(rows)
print(R.round(3).to_string(index=False, max_rows=40))
ok = R.icp_rel_scale.notna()
print(f"\nraw clouds: relative scale k->k+3 from ICP: spread {np.std(np.log(R.icp_rel_scale[ok]))*100:.1f} % (frame-to-frame scale change of the MODEL)")
print(f"our ground-based scale ratio:            spread {np.std(np.log(R.ground_ratio[ok]))*100:.1f} %")
print(f"raw predicted camera height above the cloud's ground: median {R.raw_cam_height_k.median():.3f}, spread {R.raw_cam_height_k.std()/R.raw_cam_height_k.median()*100:.1f} %")
print(f"corr(log icp scale, log ground ratio) = {np.corrcoef(np.log(R.icp_rel_scale[ok]), np.log(R.ground_ratio[ok]))[0,1]:+.2f}")
