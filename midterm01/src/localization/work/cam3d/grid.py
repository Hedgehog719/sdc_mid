"""Camera point cloud (structure only) vs LiDAR map structure: scale from the cloud's ground height (camera 1.70 m),
pitch/roll from the cloud's ground plane, then a grid search over (fwd, lat, yaw) of the inlier count."""
import numpy as np
from scipy.spatial import cKDTree
from icp import read_ply, voxel

H = 1.70

def cloud_struct(Q, rmin=3, rmax=30):
    r = np.hypot(Q[:, 0], Q[:, 1])
    Q = Q[(r > rmin) & (r < rmax)]
    near = (Q[:, 0] > 3) & (Q[:, 0] < 15) & (np.abs(Q[:, 1]) < 3)
    G = Q[near]
    if len(G) < 100: return None
    z0 = np.percentile(G[:, 2], 20)
    g = G[np.abs(G[:, 2] - z0) < 0.25]
    A = np.c_[g[:, 0], g[:, 1], np.ones(len(g))]
    a, b, c = np.linalg.lstsq(A, g[:, 2], rcond=None)[0]     # ground plane z = a x + b y + c (camera frame)
    s = H / max(-c, 0.5)
    hz = Q[:, 2] - (a * Q[:, 0] + b * Q[:, 1] + c)              # height above the cloud's ground
    S = Q[(hz > 0.3) & (hz < 6.0)] * s
    S[:, 2] = (hz[(hz > 0.3) & (hz < 6.0)]) * s                 # z = height above ground (pitch/roll removed)
    return voxel(S, 0.2), s

def map_struct(P, x, y, R=40):
    m = (np.abs(P[:, 0] - x) < R) & (np.abs(P[:, 1] - y) < R)
    L = voxel(np.asarray(P[m], dtype=np.float64), 0.1)
    # local ground per 2 m cell (10th pct), height above it
    k = np.floor(L[:, :2] / 2.0).astype(np.int64)
    key = k[:, 0] * 100000 + k[:, 1]
    o = np.lexsort((L[:, 2], key)); ks = key[o]
    u, f, n = np.unique(ks, return_index=True, return_counts=True)
    g = L[o][:, 2][f + (0.1 * (n - 1)).astype(int)]
    gz = np.empty(len(L)); gz[o] = np.repeat(g, n)
    hz = L[:, 2] - gz
    S = L[(hz > 0.3) & (hz < 6.0)].copy(); S[:, 2] = hz[(hz > 0.3) & (hz < 6.0)]
    return S

def search(S, tree, x, y, yaw, span=1.2, step=0.1, yspan=1.5, ystep=0.25, tol=0.15):
    best = None
    offs = np.arange(-span, span + 1e-9, step)
    out = {}
    for dyaw in np.radians(np.arange(-yspan, yspan + 1e-9, ystep)):
        c, s = np.cos(yaw + dyaw), np.sin(yaw + dyaw)
        W0 = np.stack([c * S[:, 0] - s * S[:, 1], s * S[:, 0] + c * S[:, 1], S[:, 2]], 1)
        grid = np.zeros((len(offs), len(offs)))
        c0, s0 = np.cos(yaw), np.sin(yaw)
        for i, f in enumerate(offs):
            for j, l in enumerate(offs):
                W = W0 + [x + c0 * f - s0 * l, y + s0 * f + c0 * l, 0]
                d, _ = tree.query(W, distance_upper_bound=tol)
                grid[i, j] = np.isfinite(d).mean()
        out[dyaw] = grid
        if best is None or grid.max() > best[0]:
            best = (grid.max(), dyaw, grid)
    return best, offs
