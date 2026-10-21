"""Segment-level forward check (no changes): the mean paint score of the projected near markings (1..15 m) as a
function of a common forward offset applied to all frames of a segment, per camera (+ background-penalised score).
A whole-segment curve is robust to per-frame ambiguity (periodic stripes) where per-frame argmax medians jump.
usage: seg_fwd_curve.py track_dir marks traj lo:hi:step"""
import sys
import numpy as np
sys.path.insert(0, 'tools')
FR = sys.argv[4]
sys.argv = ['near_align.py', sys.argv[1], sys.argv[2], sys.argv[3], '--frames', FR, '--rmax', '15', '--lat', '8']
exec(open('tools/near_align.py').read().split("if __name__ == '__main__':")[0].replace('__file__', "'tools/near_align.py'"))
import cv2
cv2.setNumThreads(1)
from multiprocessing import Pool
offs = np.round(np.arange(-0.6, 0.6001, 0.05), 3)
rng = np.random.default_rng(0)
def samp(c, q, resp):
    uv, ok = C.project(c, q @ Rm); h, w = resp[c].shape
    u, v = uv[:, 0], uv[:, 1]; ins = ok & (u >= 0) & (u < w - 1) & (v >= 0) & (v < h - 1)
    if ins.sum() < 30: return np.nan
    return float(cv2.remap(resp[c], u[ins].astype(np.float32).reshape(-1, 1), v[ins].astype(np.float32).reshape(-1, 1), cv2.INTER_LINEAR).mean())
def work(j):
    TR = TRS[0]; x, y, yaw = TR.x[j], TR.y[j], TR.yaw[j]
    P = MK.points(x, y, 20)
    if not len(P): return None
    q = vehicle_frame(P, (x, y, yaw), MK.height)
    q = q[road_level(q) & (q[:, 0] > 1) & (np.hypot(q[:, 0], q[:, 1]) < 15)]
    Bw = MK.background(x, y, 20, 3000, rng)
    qb = vehicle_frame(Bw, (x, y, yaw), MK.height)
    qb = qb[road_level(qb, tol=0.15, slope=0.02) & (qb[:, 0] > 1) & (np.hypot(qb[:, 0], qb[:, 1]) < 15)]
    resp = {c: response(c, j) for c in CAMS}
    out = np.full((len(CAMS), 2, len(offs)), np.nan)
    for ci, c in enumerate(CAMS):
        for k, df in enumerate(offs):
            sm = samp(c, q - [df, 0, 0], resp)
            sb = samp(c, qb - [df, 0, 0], resp)
            out[ci, 0, k], out[ci, 1, k] = sm, sm - sb
    return out
lo, hi, st = (int(v) for v in FR.split(':'))
with Pool(4) as p:
    R = np.array([r for r in p.map(work, range(lo, hi, st)) if r is not None])
print(f'{T} {sys.argv[3]} frames {FR}: best common forward correction (m) of the segment')
for ci, c in enumerate(CAMS):
    for si, nm in enumerate(('marks', 'marks-bg')):
        m = np.nanmean(R[:, ci, si, :], 0)
        print(f'  {c:2s} {nm:8s} best {offs[np.nanargmax(m)]:+.2f}  curve ' + ' '.join(f'{v:.3f}' for v in m[::2]))
tot = np.nansum(np.nanmean(R[:, :, 1, :], 0), 0)
print(f'  all cams marks-bg best {offs[np.nanargmax(tot)]:+.2f}')
