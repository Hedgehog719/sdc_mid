"""Forward brute force, step 2 (window search + diagnostic): sum the per-frame score curves over a window, the best
common forward offset per window = the pipeline's forward correction. Diagnostic only: compared with a reference
trajectory's forward difference (global correlation / rms; nothing of the reference enters any output).
usage: fwd_brute_eval.py fb.npz traj.csv ref.csv"""
import sys
import numpy as np, pandas as pd
z = np.load(sys.argv[1]); F = pd.read_csv(sys.argv[2]); B = pd.read_csv(sys.argv[3])
fr, Cv, offs = z['frames'], z['curves'], z['offs']; cams = list(z['cams'])
yw = F.yaw.values; dx, dy = B.x.values - F.x.values, B.y.values - F.y.values
ref = np.cos(yw) * dx + np.sin(yw) * dy
def best_of(c):
    if not np.isfinite(c).any(): return np.nan, 0.0
    k = int(np.nanargmax(c))
    if k in (0, len(c) - 1): return np.nan, 0.0
    y0, y1, y2 = c[k - 1], c[k], c[k + 1]; den = y0 - 2 * y1 + y2
    if not (den < 0): return np.nan, 0.0
    return offs[k] + 0.5 * (y0 - y2) / den * (offs[1] - offs[0]), -den
def run(W, step, sel_c, sel_b, wbg, lo=330, verbose=False):
    ci = [cams.index(c) for c in sel_c]
    S = Cv[:, ci][:, :, sel_b]                       # frames, cam, band, kind, off
    sc = S[:, :, :, 0, :] - wbg * S[:, :, :, 1, :]    # frames, cam, band, off
    # per frame: mean over available (cam, band) curves, each centred (remove its mean -> shape only)
    sc = sc - np.nanmean(sc, axis=-1, keepdims=True)
    per = np.nanmean(sc.reshape(len(fr), -1, len(offs)), axis=1)
    rows = []
    for c0 in range(lo, int(fr.max()) - W // 2, step):
        m = (fr >= c0 - W // 2) & (fr < c0 + W // 2)
        if m.sum() < 5: continue
        b, curv = best_of(np.nanmean(per[m], 0))
        rows.append((c0, b, curv, ref[max(c0 - W // 2, 0):c0 + W // 2].mean()))
    R = np.array(rows); ok = np.isfinite(R[:, 1])
    b, r = R[ok, 1], R[ok, 3]
    corr = np.corrcoef(b, r)[0, 1] if ok.sum() > 3 else np.nan
    # explained: rms(ref) vs rms(ref - best) and vs rms(ref - k*best) with k = regression slope (diagnostic)
    k = np.sum(b * r) / np.sum(b * b)
    return dict(n=ok.sum(), nall=len(R), corr=corr, rms_ref=np.sqrt(np.mean(r ** 2)), rms_after=np.sqrt(np.mean((r - b) ** 2)),
                slope=k, mean_b=b.mean(), rms_b=np.sqrt(np.mean(b ** 2)), R=R)
print('cams        bands   bg   W    n/all  corr  rms(ref) rms(ref-best) slope  mean(best) rms(best)')
for sel_c in (['f'], ['fl', 'f', 'fr'], ['fl', 'f']):
    for sel_b, bn in (([0], 'near'), ([1], 'mid'), ([0, 1], 'both')):
        for wbg in (0.0, 1.0):
            for W in (45, 90, 180):
                d = run(W, 15, sel_c, sel_b, wbg)
                print(f'{",".join(sel_c):11s} {bn:5s} {wbg:3.1f} {W:4d} {d["n"]:3d}/{d["nall"]:3d} {d["corr"]:+.2f}   {d["rms_ref"]:.3f}    {d["rms_after"]:.3f}      {d["slope"]:+.2f}   {d["mean_b"]:+.3f}     {d["rms_b"]:.3f}')
