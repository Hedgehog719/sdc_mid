"""Register the per-frame camera point cloud (MapAnything, front-camera FLU, unknown scale) to the LiDAR map:
trimmed similarity ICP (scale, rotation, translation) started at a prior pose; returns the camera pose in the map."""
import numpy as np, pandas as pd, sys
from scipy.spatial import cKDTree

def read_ply(p):
    with open(p, 'rb') as f:
        n = None
        while True:
            l = f.readline().decode().strip()
            if l.startswith('element vertex'): n = int(l.split()[-1])
            if l == 'end_header': break
        d = np.frombuffer(f.read(n * 15), dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('r','u1'),('g','u1'),('b','u1')])
    return np.stack([d['x'], d['y'], d['z']], 1).astype(np.float64)

def voxel(Q, v):
    k = np.floor(Q / v).astype(np.int64)
    _, i = np.unique(k, axis=0, return_index=True)
    return Q[i]

def umeyama(A, B, w):
    w = w / w.sum(); ma, mb = w @ A, w @ B
    A0, B0 = A - ma, B - mb
    C = (B0 * w[:, None]).T @ A0
    U, S, Vt = np.linalg.svd(C); D = np.eye(3); D[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ D @ Vt
    s = np.trace(np.diag(S) @ D) / (w @ np.sum(A0 ** 2, 1))
    return s, R, mb - s * R @ ma

def register(Q, tree, R0, t0, s0, iters=30):
    s, R, t = s0, R0, t0
    for it in range(iters):
        dmax = 1.5 if it < 8 else (0.7 if it < 16 else 0.4)
        W = s * Q @ R.T + t
        d, j = tree.query(W, distance_upper_bound=dmax)
        ok = np.isfinite(d)
        if ok.sum() < 200: return None
        w = 1.0 / (1.0 + (d[ok] / 0.2) ** 2)
        s, R, t = umeyama(Q[ok], tree.data[j[ok]], w)
    W = s * Q @ R.T + t
    d, _ = tree.query(W, distance_upper_bound=0.5)
    return s, R, t, float(np.mean(d < 0.2)), float(np.median(np.where(np.isfinite(d), d, 0.5)))
