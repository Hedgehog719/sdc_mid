"""Forward brute force, step 1 (data): per frame, the paint score of the projected road markings as a function of a
forward offset of the pose, per camera and range band, marks and background separately (offline window search).
usage: fwd_brute.py track_dir marks_D.npz traj.csv lo:hi:step out.npz"""
import sys
import numpy as np
sys.path.insert(0, 'tools')
FR, OUT = sys.argv[4], sys.argv[5]
sys.argv = ['near_align.py', sys.argv[1], sys.argv[2], sys.argv[3], '--frames', FR, '--rmax', '16', '--lat', '6']
exec(open('tools/near_align.py').read().split("if __name__ == '__main__':")[0].replace('__file__', "'tools/near_align.py'"))
import cv2
cv2.setNumThreads(1)
from multiprocessing import Pool
OFFS = np.round(np.arange(-0.6, 0.6001, 0.02), 3)
BANDS = [(1.0, 8.0), (8.0, 16.0)]
def samp(c, q, R):
    if len(q) < 10: return np.nan
    uv, ok = C.project(c, q @ Rm); h, w = R.shape
    u, v = uv[:, 0], uv[:, 1]; ins = ok & (u >= 0) & (u < w - 1) & (v >= 0) & (v < h - 1)
    if ins.sum() < 10: return np.nan
    return float(cv2.remap(R, u[ins].astype(np.float32).reshape(-1, 1), v[ins].astype(np.float32).reshape(-1, 1), cv2.INTER_LINEAR).mean())
def work(j):
    TR = TRS[0]; x, y, yaw = TR.x[j], TR.y[j], TR.yaw[j]
    out = np.full((len(CAMS), len(BANDS), 2, len(OFFS)), np.nan, np.float32)
    P = MK.points(x, y, 22)
    if not len(P): return j, out
    q = vehicle_frame(P, (x, y, yaw), MK.height); q = q[road_level(q) & (np.abs(q[:, 1]) < 6)]
    Bw = MK.background(x, y, 22, 4000, np.random.default_rng(j)); qb = vehicle_frame(Bw, (x, y, yaw), MK.height)
    qb = qb[road_level(qb, tol=0.15, slope=0.02) & (np.abs(qb[:, 1]) < 6)]
    for ci, c in enumerate(CAMS):
        R = response(c, j)
        for bi, (r0, r1) in enumerate(BANDS):
            qm = q[(q[:, 0] >= r0) & (q[:, 0] < r1)]; qbb = qb[(qb[:, 0] >= r0) & (qb[:, 0] < r1)]
            if len(qm) < 10 or len(qbb) < 10: continue
            for k, df in enumerate(OFFS):
                sh = np.array([df, 0, 0])
                out[ci, bi, 0, k] = samp(c, qm - sh, R); out[ci, bi, 1, k] = samp(c, qbb - sh, R)
    return j, out
lo, hi, st = (int(v) for v in FR.split(':'))
hi = len(TRS[0]) if hi < 0 else hi
with Pool(4) as p:
    res = p.map(work, range(lo, hi, st), chunksize=4)
np.savez(OUT, frames=np.array([r[0] for r in res]), curves=np.stack([r[1] for r in res]), offs=OFFS,
         bands=np.array(BANDS), cams=np.array(CAMS), traj=sys.argv[3])
print(f'{T}: {len(res)} frames -> {OUT}')
