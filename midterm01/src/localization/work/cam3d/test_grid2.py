import sys, time, numpy as np, pandas as pd
sys.path.insert(0, 'work/cam3d')
from icp import read_ply
from grid import cloud_struct, map_struct
from scipy.spatial import cKDTree
T = sys.argv[1]; frames = [int(v) for v in sys.argv[2].split(',')]
best = {'Track2': 'submissions/track2/0.17429__t2_opt10.csv', 'Track1': 'submissions/track1/0.41123__t1_opt11.csv'}[T]
idx = pd.read_csv(f'/root/data/{T}/3cameras_ply/index.csv'); tr = pd.read_csv(best)
M = np.load(f'/root/data/{T}/map_clean.npy', mmap_mode='r')
offs = np.arange(-1.0, 1.0001, 0.1)
for fr in frames:
    t0 = time.time()
    S, sc = cloud_struct(read_ply(f'/root/data/{T}/3cameras_ply/' + idx.ply[fr]), rmax=15)
    x, y, yaw = tr.x[fr], tr.y[fr], tr.yaw[fr]
    tree = cKDTree(map_struct(M, x, y, R=25))
    res = []
    for k in (0.92, 0.96, 1.0, 1.04, 1.08):
        Sk = S.copy(); Sk[:, :2] *= k
        for dyaw in np.radians(np.arange(-1.0, 1.01, 0.5)):
            c, s = np.cos(yaw + dyaw), np.sin(yaw + dyaw); c0, s0 = np.cos(yaw), np.sin(yaw)
            W0 = np.stack([c * Sk[:, 0] - s * Sk[:, 1], s * Sk[:, 0] + c * Sk[:, 1], Sk[:, 2]], 1)
            g = np.zeros((len(offs), len(offs)))
            for i, f in enumerate(offs):
                for j, l in enumerate(offs):
                    d, _ = tree.query(W0 + [x + c0 * f - s0 * l, y + s0 * f + c0 * l, 0], distance_upper_bound=0.15)
                    g[i, j] = np.isfinite(d).mean()
            i, j = np.unravel_index(np.argmax(g), g.shape)
            res.append((g.max(), k, np.degrees(dyaw), offs[i], offs[j], g))
    res.sort(key=lambda r: -r[0])
    pk, k, dy, f, l, g = res[0]
    second = next((r for r in res[1:] if abs(r[3] - f) > 0.25 or abs(r[4] - l) > 0.25), None)
    print(f'{T} f{fr}: n {len(S)} best peak {pk:.3f} scale x{k} dyaw {dy:+.1f} fwd {f:+.1f} lat {l:+.1f} | best distinct alternative {second[0]:.3f} at fwd {second[3]:+.1f} lat {second[4]:+.1f} | {time.time()-t0:.0f}s')
