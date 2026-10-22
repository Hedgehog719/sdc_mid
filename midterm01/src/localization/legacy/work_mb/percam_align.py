"""Per-camera alignment diagnosis (no changes): for each camera separately, the (fwd, lat) shift of the projected
road markings (1..15 m) that best matches that camera's paint. Cameras agreeing -> pose error; one camera off ->
that camera's calibration; summarised per segment."""
import os, sys
import numpy as np, pandas as pd
sys.path.insert(0, 'tools')
FRAMES = sys.argv[4]
sys.argv = ['near_align.py', sys.argv[1], sys.argv[2], sys.argv[3], '--frames', FRAMES, '--rmax', '15', '--lat', '8']
src = open('tools/near_align.py').read().split("if __name__ == '__main__':")[0].replace('__file__', "'tools/near_align.py'")
exec(src)
import cv2
cv2.setNumThreads(1)
from multiprocessing import Pool
offs = np.arange(-0.8, 0.8001, 0.1)
def cam_score(c, q, resp):
    uv, ok = C.project(c, q @ Rm); h, w = resp[c].shape
    u, v = uv[:, 0], uv[:, 1]; ins = ok & (u >= 0) & (u < w - 1) & (v >= 0) & (v < h - 1)
    if ins.sum() < 40: return np.nan
    return float(cv2.remap(resp[c], u[ins].astype(np.float32).reshape(-1, 1), v[ins].astype(np.float32).reshape(-1, 1), cv2.INTER_LINEAR).mean())
def work(j):
    TR = TRS[0]; x, y, yaw = TR.x[j], TR.y[j], TR.yaw[j]
    P = MK.points(x, y, 20)
    if not len(P): return None
    q = vehicle_frame(P, (x, y, yaw), MK.height)
    q = q[road_level(q) & (q[:, 0] > -3) & (np.hypot(q[:, 0], q[:, 1]) < 15)]
    resp = {c: response(c, j) for c in CAMS}
    out = {'frame': j}
    for c in CAMS:
        g = np.full((len(offs), len(offs)), np.nan)
        for i, df in enumerate(offs):
            for k, dl in enumerate(offs):
                g[i, k] = cam_score(c, q - [df, dl, 0], resp)
        if np.isfinite(g).any() and np.nanmax(g) > 0.05:
            i, k = np.unravel_index(np.nanargmax(g), g.shape)
            edge = i in (0, len(offs) - 1) or k in (0, len(offs) - 1)
            out[c] = (offs[i], offs[k], edge)
    return out
lo, hi, st = (int(v) for v in FRAMES.split(':'))
with Pool(4) as p:
    R = [r for r in p.map(work, range(lo, len(TRS[0]) if hi < 0 else hi, st)) if r]
print(f'{T}: per-camera best map shift = pose/camera correction (fwd, lat) in m, median per segment (non-edge)')
n = len(TRS[0])
for s0 in range(0, n, 300):
    row = f'  {s0:4d}:'
    for c in CAMS:
        v = np.array([r[c][:2] for r in R if c in r and not r[c][2] and s0 <= r['frame'] < s0 + 300])
        row += f'  {c:2s} ' + (f'{np.median(v[:,0]):+.2f}/{np.median(v[:,1]):+.2f} (n{len(v):2d})' if len(v) >= 3 else '   -   ')
    print(row)
