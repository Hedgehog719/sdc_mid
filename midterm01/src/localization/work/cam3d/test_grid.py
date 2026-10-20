import sys, time, numpy as np, pandas as pd
sys.path.insert(0, 'work/cam3d')
from icp import read_ply
from grid import cloud_struct, map_struct, search
from scipy.spatial import cKDTree
T = sys.argv[1]; frames = [int(v) for v in sys.argv[2].split(',')]
best = {'Track2': 'submissions/track2/0.17429__t2_opt10.csv', 'Track1': 'submissions/track1/0.41123__t1_opt11.csv'}[T]
idx = pd.read_csv(f'/root/data/{T}/3cameras_ply/index.csv'); tr = pd.read_csv(best)
M = np.load(f'/root/data/{T}/map_clean.npy', mmap_mode='r')
for fr in frames:
    t0 = time.time()
    cs = cloud_struct(read_ply(f'/root/data/{T}/3cameras_ply/' + idx.ply[fr]))
    if cs is None: print(fr, 'no ground'); continue
    S, sc = cs
    x, y, yaw = tr.x[fr], tr.y[fr], tr.yaw[fr]
    tree = cKDTree(map_struct(M, x, y))
    (pk, dyaw, g), offs = search(S, tree, x, y, yaw)
    i, j = np.unravel_index(np.argmax(g), g.shape)
    # peak sharpness: drop of the score 0.3 m away along each axis
    sf = pk - max(g[max(i - 3, 0), j], g[min(i + 3, len(offs) - 1), j]); sl = pk - max(g[i, max(j - 3, 0)], g[i, min(j + 3, len(offs) - 1)])
    print(f'{T} f{fr}: scale {sc:.3f} n {len(S)} peak {pk:.3f} at fwd {offs[i]:+.1f} lat {offs[j]:+.1f} dyaw {np.degrees(dyaw):+.2f} | sharp fwd {sf:.3f} lat {sl:.3f} | {time.time()-t0:.1f}s')
