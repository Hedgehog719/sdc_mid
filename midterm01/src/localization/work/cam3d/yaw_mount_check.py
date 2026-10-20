"""Yaw offset between the camera cloud (front FLU) and the vehicle heading: scan the cloud's yaw at the pipeline pose,
score = share of cloud structure points within 0.15 m of map structure (position fixed)."""
import sys, numpy as np, pandas as pd
from scipy.spatial import cKDTree
sys.path.insert(0, 'work/cam3d'); sys.path.insert(0, 'tools')
from icp import read_ply, voxel
from grid import cloud_struct, map_struct
T = sys.argv[1]; V = pd.read_csv(sys.argv[2]); tag = sys.argv[3]
idx = pd.read_csv(f'/root/data/{T}/3cameras_ply/index.csv')
M = np.load(f'/root/data/{T}/map_{tag}.npy', mmap_mode='r')
offs = np.arange(-4, 4.01, 0.25)
best = []
for fr in range(450, len(V) - 10, 150):
    cs = cloud_struct(read_ply(f'/root/data/{T}/3cameras_ply/' + idx.ply[fr]), rmax=20)
    if cs is None: continue
    S, sc = cs
    x, y, yaw = V.x[fr], V.y[fr], V.yaw[fr]
    tree = cKDTree(map_struct(M, x, y, R=30))
    sco = []
    for o in offs:
        c, s = np.cos(yaw + np.radians(o)), np.sin(yaw + np.radians(o))
        W = np.stack([x + c * S[:, 0] - s * S[:, 1], y + s * S[:, 0] + c * S[:, 1], S[:, 2]], 1)
        d, _ = tree.query(W, distance_upper_bound=0.15)
        sco.append(np.isfinite(d).mean())
    sco = np.array(sco); best.append(offs[np.argmax(sco)])
    print(f'{T} f{fr}: best cloud yaw offset {offs[np.argmax(sco)]:+.2f} deg (score {sco.max():.3f}, at 0: {sco[16]:.3f}, at -1.5: {sco[10]:.3f})')
print(f'median best offset {np.median(best):+.2f} deg over {len(best)} frames')
