"""Range-band forward check (no changes): for one camera, the forward offset of the projected markings that best
matches the paint, computed separately per forward-distance band (whole-run mean curves). A pitch error shifts a
ground point radially by ~r^2/h * dtheta, a height error by ~r/h * dh: a consistent trend of the best offset with the
band distance measures them (a common offset = pose).
usage: band_fwd.py track_dir marks traj lo:hi:step cam"""
import sys
import numpy as np
sys.path.insert(0, 'tools')
FR, CAM = sys.argv[4], sys.argv[5]
sys.argv = ['near_align.py', sys.argv[1], sys.argv[2], sys.argv[3], '--frames', FR, '--rmax', '22', '--lat', '4']
exec(open('tools/near_align.py').read().split("if __name__ == '__main__':")[0].replace('__file__', "'tools/near_align.py'"))
import cv2
cv2.setNumThreads(1)
from multiprocessing import Pool
offs = np.round(np.arange(-0.8, 0.8001, 0.04), 3)
BANDS = [(2, 5), (5, 8), (8, 11), (11, 14), (14, 18), (18, 22)]
rng = np.random.default_rng(0)
def samp(q, R):
    uv, ok = C.project(CAM, q @ Rm); h, w = R.shape
    u, v = uv[:, 0], uv[:, 1]; ins = ok & (u >= 0) & (u < w - 1) & (v >= 0) & (v < h - 1)
    if ins.sum() < 15: return np.nan
    return float(cv2.remap(R, u[ins].astype(np.float32).reshape(-1, 1), v[ins].astype(np.float32).reshape(-1, 1), cv2.INTER_LINEAR).mean())
def work(j):
    TR = TRS[0]; x, y, yaw = TR.x[j], TR.y[j], TR.yaw[j]
    P = MK.points(x, y, 30)
    if not len(P): return None
    q = vehicle_frame(P, (x, y, yaw), MK.height); q = q[road_level(q) & (np.abs(q[:, 1]) < 4)]
    Bw = MK.background(x, y, 30, 4000, rng); qb = vehicle_frame(Bw, (x, y, yaw), MK.height)
    qb = qb[road_level(qb, tol=0.15, slope=0.02) & (np.abs(qb[:, 1]) < 4)]
    R = response(CAM, j)
    out = np.full((len(BANDS), len(offs)), np.nan)
    for b, (r0, r1) in enumerate(BANDS):
        qm, qbb = q[(q[:, 0] >= r0) & (q[:, 0] < r1)], qb[(qb[:, 0] >= r0) & (qb[:, 0] < r1)]
        if len(qm) < 15 or len(qbb) < 15: continue
        for k, df in enumerate(offs):
            out[b, k] = samp(qm - [df, 0, 0], R) - samp(qbb - [df, 0, 0], R)
    return out
lo, hi, st = (int(v) for v in FR.split(':'))
with Pool(4) as p:
    Rs = np.array([r for r in p.map(work, range(lo, hi, st)) if r is not None])
print(f'{T} {CAM} {sys.argv[3]} frames {FR}: best forward offset per band (whole-run mean curve, marks - background)')
res = []
for b, (r0, r1) in enumerate(BANDS):
    m = np.nanmean(Rs[:, b, :], 0); n = np.isfinite(Rs[:, b, 0]).sum()
    k = int(np.nanargmax(m))
    if 0 < k < len(offs) - 1:   # parabolic refinement
        y0, y1, y2 = m[k - 1], m[k], m[k + 1]; den = y0 - 2 * y1 + y2
        best = offs[k] + (0.5 * (y0 - y2) / den * 0.04 if den < 0 else 0)
    else:
        best = offs[k]
    res.append(((r0 + r1) / 2, best, n))
    print(f'  {r0:2d}-{r1:2d} m: best {best:+.3f} (n {n}, peak {np.nanmax(m):.3f})')
r = np.array([v[0] for v in res]); d = np.array([v[1] for v in res])
A = np.stack([np.ones_like(r), r, r ** 2], 1)
c = np.linalg.lstsq(A, d, rcond=None)[0]
print(f'  fit best(r) = {c[0]:+.3f} {c[1]:+.4f} r {c[2]:+.5f} r^2')
