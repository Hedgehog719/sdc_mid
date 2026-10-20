"""Step between synthetic start ground and the real ground around it (T2 start), per lateral bin."""
import sys, numpy as np, pandas as pd
from scipy.spatial import cKDTree
tag = sys.argv[1]
P = np.load(f'/root/data/Track2/map_{tag}.npy', mmap_mode='r')
F = np.load(f'/root/data/Track2/filled_{tag}.npy')
tr = pd.read_csv('submissions/track2/0.17429__t2_opt10.csv'); x, y = tr.x.values, tr.y.values
s_cum = np.r_[0, np.cumsum(np.hypot(np.diff(x), np.diff(y)))]
k0 = int(np.searchsorted(s_cum, 3.0)); hx, hy = x[k0]-x[0], y[k0]-y[0]; hn = np.hypot(hx, hy); hx, hy = hx/hn, hy/hn
qs = np.arange(-30, 60, 0.5)
px = np.where(qs < 0, x[0]+qs*hx, np.interp(np.clip(qs,0,None), s_cum, x)); py = np.where(qs < 0, y[0]+qs*hy, np.interp(np.clip(qs,0,None), s_cum, y))
gx, gy = np.gradient(px), np.gradient(py); gn = np.hypot(gx, gy); gx, gy = gx/gn, gy/gn
tree = cKDTree(np.stack([px, py], 1))
def sl(Q):
    d, j = tree.query(Q[:, :2]); return qs[j] + gx[j]*(Q[:,0]-px[j]) + gy[j]*(Q[:,1]-py[j]), -gy[j]*(Q[:,0]-px[j]) + gx[j]*(Q[:,1]-py[j])
m = (np.abs(P[:,0]-px[60]) < 45) & (np.abs(P[:,1]-py[60]) < 45)
Q = np.asarray(P[m], dtype=np.float64)
syn = np.zeros(len(P), bool); syn[len(P) - len(F):] = True     # synthetic points are appended at the end
isf = syn[m]
sq, lq = sl(Q); sf, lf = sl(F)
zf = F[:, 2]
print(f'{tag}: {isf.sum()} synthetic points in the window (file {len(F)})')
print(' l bin | front seam: synthetic (last 0.5 m) vs real ground (next 1 m) | real ground just behind the fill?')
for l0 in np.arange(-6, 5.01, 1.0):
    ms = (np.abs(lf - l0) < 0.5)
    if not ms.any(): continue
    s_edge = sf[ms].max()
    syn = zf[ms & (sf > s_edge - 0.5)]   # noqa
    mr = (~isf) & (np.abs(lq - l0) < 0.5) & (sq > s_edge) & (sq < s_edge + 1.0)
    zr = Q[mr, 2]; zr = zr[np.abs(zr - np.median(syn)) < 0.3]
    print(f' l {l0:+4.1f}: fill ends at s {s_edge:5.1f}; synthetic {np.median(syn):.3f}, real {np.median(zr) if len(zr) else np.nan:.3f} (n {len(zr)}) -> step {np.median(zr) - np.median(syn) if len(zr) else np.nan:+.3f} m')
for side, lsel in (('right side l -7..-6', (-7, -6)), ('left side l +5..+6', (5, 6))):
    for s0 in (-25, -15, -8, -2):
        mr = (~isf) & (lq > lsel[0]) & (lq < lsel[1]) & (np.abs(sq - s0) < 1.0)
        msy = (lf > lsel[0] + (1 if lsel[0] < 0 else -1)) & (lf < lsel[1] + (1 if lsel[0] < 0 else -1)) & (np.abs(sf - s0) < 1.0)
        zr = Q[mr, 2]
        print(f' {side} s {s0:+4d}: real p10 {np.percentile(zr,10) if len(zr) > 5 else np.nan:.3f} median {np.median(zr) if len(zr) > 5 else np.nan:.3f} (n {len(zr)}), adjacent synthetic {np.median(zf[msy]) if msy.any() else np.nan:.3f}')
