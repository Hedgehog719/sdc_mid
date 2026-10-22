#!/usr/bin/env python3
"""Remove vehicles standing on the road (car roofs) from the map along the whole driven path.

Where a vehicle hid the ground completely while mapping (e.g. at the Track2 start: no ground under the car, only its
roof at +1.4..2.8 m), every ground estimate (per-cell minimum, grey opening) takes the roof as ground, so the road
under the vehicle tilts by up to 13 deg. Here, in path coordinates (s along the path, l left), per 2 m cell the
10th-percentile height is taken; per --win m of path a lower-envelope plane is fitted to the cells with |l| < --fit_half
(iterative: cells more than --env above the plane are dropped, roofs are always above the road); then points with
|l| < --road_half between plane + --clear_h and + --max_h are deleted (vehicles, people; poles and signs stand beside the road).
Writes map_<tag_out>.npy + intensity, and prints the cleared segments.

usage: clear_roofs.py <track_dir> <traj_csv> --tag_in clean --tag_out clean12
"""
import argparse
import os

import numpy as np
import pandas as pd
from scipy import sparse
from scipy.ndimage import binary_dilation, map_coordinates
from scipy.sparse.linalg import spsolve
from scipy.spatial import cKDTree

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('traj_csv')
ap.add_argument('--tag_in', default='clean')
ap.add_argument('--tag_out', required=True)
ap.add_argument('--win', type=float, default=12.0, help='m of path per plane')
ap.add_argument('--fit_half', type=float, default=8.0)
ap.add_argument('--road_half', type=float, default=5.0)
ap.add_argument('--l_left', type=float, default=None, help='clear up to this far left of the path (default road_half)')
ap.add_argument('--l_right', type=float, default=None, help='clear up to this far right of the path (default road_half)')
ap.add_argument('--fill_to', type=float, default=0.0,
                help='>0: for s < fill_to the reference road is the plane fitted on --trust (used to classify ground / '
                     'objects); the hole without real ground is filled by Laplace interpolation from the real road around it')
ap.add_argument('--trust', default='8:30')
ap.add_argument('--fill_left', type=float, default=6.0)
ap.add_argument('--start_clear', type=float, default=0.0,
                help='>0: for s < start_clear delete EVERYTHING higher than road + --start_h in the band '
                     '-start_right..+start_left (vehicles standing next to / in front of the car at the start, scan streaks)')
ap.add_argument('--start_left', type=float, default=3.8)
ap.add_argument('--start_right', type=float, default=1.5)
ap.add_argument('--start_h', type=float, default=0.15)
ap.add_argument('--clip_intensity', type=float, default=255.0, help='intensities above this are invalid (86400 sentinels) -> clipped')
ap.add_argument('--fill_right', type=float, default=6.0)
ap.add_argument('--clear_h', type=float, default=1.0)
ap.add_argument('--max_h', type=float, default=2.6, help='keep points higher than road + max_h (signs, bridges, tree crowns)')
ap.add_argument('--env', type=float, default=0.25)
ap.add_argument('--back', type=float, default=30.0, help='m before the start along the initial heading')
a = ap.parse_args()

sfx = f'_{a.tag_in}' if a.tag_in else ''
P = np.load(os.path.join(a.track_dir, f'map{sfx}.npy'), mmap_mode='r')
I = np.load(os.path.join(a.track_dir, f'map_intensity{sfx}.npy'), mmap_mode='r')
tr = pd.read_csv(a.traj_csv)
x, y = tr.x.values, tr.y.values
s_cum = np.r_[0, np.cumsum(np.hypot(np.diff(x), np.diff(y)))]
k0 = int(np.searchsorted(s_cum, 3.0))
hx, hy = x[k0] - x[0], y[k0] - y[0]
hn = np.hypot(hx, hy)
hx, hy = hx / hn, hy / hn
qs = np.arange(-a.back, s_cum[-1] + 5, 0.5)
px = np.where(qs < 0, x[0] + qs * hx, np.interp(np.clip(qs, 0, None), s_cum, x))
py = np.where(qs < 0, y[0] + qs * hy, np.interp(np.clip(qs, 0, None), s_cum, y))
gx, gy = np.gradient(px), np.gradient(py)
gn = np.hypot(gx, gy)
gx, gy = np.where(gn > 0, gx / np.maximum(gn, 1e-9), np.nan), np.where(gn > 0, gy / np.maximum(gn, 1e-9), np.nan)
gx, gy = pd.Series(gx).ffill().bfill().values, pd.Series(gy).ffill().bfill().values
xyz = np.asarray(P, dtype=np.float64)
d, j = cKDTree(np.stack([px, py], 1)).query(xyz[:, :2], distance_upper_bound=a.fit_half + 2)
inr = np.isfinite(d)
idx = np.flatnonzero(inr)
jj = j[inr]
sp = qs[jj] + gx[jj] * (xyz[idx, 0] - px[jj]) + gy[jj] * (xyz[idx, 1] - py[jj])
lp = -gy[jj] * (xyz[idx, 0] - px[jj]) + gx[jj] * (xyz[idx, 1] - py[jj])
zp = xyz[idx, 2]
cs = np.floor((sp + a.back) / 2.0).astype(int)
cl = np.floor((lp + a.fit_half + 2) / 2.0).astype(int)
nl = cl.max() + 1
key = cs * nl + cl
o = np.lexsort((zp, key))
u, first, cnt = np.unique(key[o], return_index=True, return_counts=True)
gz = zp[o][first + (0.1 * (cnt - 1)).astype(int)]
gs = (u // nl + 0.5) * 2.0 - a.back
gl = (u % nl + 0.5) * 2.0 - a.fit_half - 2
ok_c = (np.abs(gl) < a.fit_half) & (cnt >= 5)
# per window: lower-envelope plane z = c0 + c1 (s - s_mid) + c2 l
nw = int(np.ceil((qs[-1] + a.back) / a.win)) + 1
planes = np.full((nw, 3), np.nan)
wid_c = np.floor((gs + a.back) / a.win).astype(int)
for w in range(nw):
    m = ok_c & (np.abs(wid_c - w) <= 1)     # 3 windows of cells for a stable fit
    if m.sum() < 8:
        continue
    sm = (w + 0.5) * a.win - a.back
    A = np.stack([np.ones(m.sum()), gs[m] - sm, gl[m]], 1)
    z = gz[m]
    keep = np.ones(len(z), bool)
    for _ in range(6):
        co = np.linalg.lstsq(A[keep], z[keep], rcond=None)[0]
        r = z - A @ co
        nk = r < a.env
        if nk.sum() < 6 or (nk == keep).all():
            break
        keep = nk
    planes[w] = co
wid = np.clip(np.floor((sp + a.back) / a.win).astype(int), 0, nw - 1)
sm = (wid + 0.5) * a.win - a.back
pl = planes[wid]
zref = pl[:, 0] + pl[:, 1] * (sp - sm) + pl[:, 2] * lp
lL = a.road_half if a.l_left is None else a.l_left
lR = a.road_half if a.l_right is None else a.l_right
if a.fill_to > 0:   # start: road plane of the trusted road after the start, extended backwards
    t0, t1 = (float(v) for v in a.trust.split(':'))
    m = ok_c & (gs >= t0) & (gs <= t1)
    A = np.stack([np.ones(m.sum()), gs[m], gl[m]], 1)
    z = gz[m]
    keep = np.ones(len(z), bool)
    for _ in range(6):
        cst = np.linalg.lstsq(A[keep], z[keep], rcond=None)[0]
        keep = (z - A @ cst) < a.env
    gb = (sp >= t0) & (sp <= t1) & (np.abs(lp) < a.fit_half)
    zr_t = cst[0] + cst[1] * sp[gb] + cst[2] * lp[gb]
    res_t = zp[gb] - zr_t
    off_t = float(np.median(res_t[np.abs(res_t) < 0.15]))    # lower-envelope plane sits below the road surface
    cst[0] += off_t
    print(f'start plane raised by {off_t:+.3f} m to the median of the trusted road points')
    print(f'start road plane (s {a.trust}): z = {cst[0]:.3f} + {cst[1] * 100:.3f} % s + {cst[2] * 100:.3f} % l')
    st = sp < a.fill_to
    zref[st] = cst[0] + cst[1] * sp[st] + cst[2] * lp[st]
above = np.isfinite(zref) & (lp < lL) & (lp > -lR) & (zp > zref + a.clear_h) & (zp < zref + a.max_h)
if a.start_clear > 0:
    stc = np.isfinite(zref) & (sp < a.start_clear) & (lp < a.start_left) & (lp > -a.start_right) & (zp > zref + a.start_h)
    print(f'start clear: {(stc & ~above).sum()} more points above road + {a.start_h} m '
          f'for s < {a.start_clear} m, l -{a.start_right}..+{a.start_left} m')
    above |= stc
drop = np.zeros(len(P), bool)
drop[idx[above]] = True
print(f'cleared {above.sum()} points ({above.sum() / len(P) * 100:.3f} %) road + {a.clear_h}..{a.max_h} m within {a.road_half} m of the path')
# report segments (per 2 m of path with > 200 cleared points)
hs = np.bincount(np.clip(((sp[above] + a.back) / 2).astype(int), 0, None), minlength=int((qs[-1] + a.back) / 2) + 2)
seg = []
for k in np.flatnonzero(hs > 200):
    s0 = k * 2 - a.back
    if seg and s0 - seg[-1][1] <= 4:
        seg[-1][1] = s0 + 2
        seg[-1][2] += hs[k]
    else:
        seg.append([s0, s0 + 2, hs[k]])
fr_of = lambda s: int(np.searchsorted(s_cum, max(s, 0)))  # noqa: E731
for s0, s1, n in seg:
    print(f'   s {s0:7.0f}..{s1:7.0f} m (frames {fr_of(s0)}-{fr_of(s1)}): {n} points')
np.save(os.path.join(a.track_dir, f'removed_{a.tag_out}_idx.npy'), np.flatnonzero(drop))   # indices into map_<tag_in>
outP, outI = np.asarray(P)[~drop], np.asarray(I)[~drop]
nbad = int((outI > a.clip_intensity).sum())
outI = np.minimum(outI, a.clip_intensity).astype(outI.dtype)
print(f'clipped {nbad} intensities above {a.clip_intensity:.0f}')
if a.fill_to > 0:
    # Ground for the start hole, continuous with the real road around it: per 0.2 m cell the median of the kept points
    # within +-0.15 m of the reference road ("known" with >= 3 points); the hole cells solve the Laplace equation
    # (membrane: each = mean of its 4 neighbours, Neumann at the grid edge), so the surface meets every real boundary
    # cell exactly and is smooth inside - no plane, no global offset. Synthetic points copy the real road next to the
    # hole: points per 0.1 m cell, height residuals about the cell median and asphalt intensities (resampled).
    R = 0.2
    S0g, S1g = -a.back - 1.0, a.fill_to + 3.0
    L0g, L1g = -a.fill_right - 1.5, a.fill_left + 1.5
    gnd = ~drop[idx] & (np.abs(zp - zref) < 0.15) & (sp >= S0g) & (sp < S1g) & (lp >= L0g) & (lp < L1g)
    nsg, nlg = int(np.ceil((S1g - S0g) / R)), int(np.ceil((L1g - L0g) / R))
    gk = ((sp[gnd] - S0g) / R).astype(int) * nlg + ((lp[gnd] - L0g) / R).astype(int)
    o = np.lexsort((zp[gnd], gk))
    gk_s, zg_s = gk[o], zp[gnd][o]
    ig_s = np.minimum(np.asarray(I)[idx[gnd]], a.clip_intensity)[o]
    uk, first, cnt = np.unique(gk_s, return_index=True, return_counts=True)
    Z = np.full(nsg * nlg, np.nan)
    C = np.zeros(nsg * nlg, int)
    Z[uk] = zg_s[first + (cnt - 1) // 2]
    C[uk] = cnt
    known = C >= 3
    unk = np.flatnonzero(~known)
    pos = -np.ones(nsg * nlg, int)
    pos[unk] = np.arange(len(unk))
    ui, uj = unk // nlg, unk % nlg
    eps = 1e-4                                   # tiny pull to the start plane: pockets without any boundary stay defined
    diag = np.full(len(unk), eps)
    rhs = eps * (cst[0] + cst[1] * (S0g + (ui + 0.5) * R) + cst[2] * (L0g + (uj + 0.5) * R))
    rows, cols = [np.arange(len(unk))], [np.arange(len(unk))]
    vals = []
    for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        ni, nj = ui + di, uj + dj
        ok = (ni >= 0) & (ni < nsg) & (nj >= 0) & (nj < nlg)
        diag[ok] += 1
        r_ok = np.flatnonzero(ok)
        nk = ni[ok] * nlg + nj[ok]
        kn = known[nk]
        np.add.at(rhs, r_ok[kn], Z[nk[kn]])
        rows.append(r_ok[~kn])
        cols.append(pos[nk[~kn]])
        vals.append(-np.ones((~kn).sum()))
    A = sparse.csr_matrix((np.concatenate([diag] + vals), (np.concatenate(rows), np.concatenate(cols))), shape=(len(unk), len(unk)))
    Z[unk] = spsolve(A, rhs)
    Zg = Z.reshape(nsg, nlg)
    # the real road next to the hole (known cells within 1 m of an unknown one): density, residuals, intensities
    Kg = known.reshape(nsg, nlg)
    ring = (Kg & binary_dilation(~Kg, iterations=5)).ravel()
    in_ring = ring[gk_s]
    resid = np.clip(zg_s[in_ring] - Z[gk_s[in_ring]], -0.1, 0.1)
    inten = ig_s[in_ring]
    inten = inten[inten <= np.percentile(inten, 85)]          # asphalt, not paint
    per01 = max(1, int(round(np.median(C[ring]) / 4)))         # points per 0.1 m cell
    occ = set(zip(np.floor(sp[gnd] / 0.1).astype(int).tolist(), np.floor(lp[gnd] / 0.1).astype(int).tolist()))
    S, L = np.meshgrid(np.arange(-a.back, a.fill_to, 0.1) + 0.05, np.arange(-a.fill_right, a.fill_left, 0.1) + 0.05, indexing='ij')
    S, L = S.ravel(), L.ravel()
    cell_known = Kg[np.clip(((S - S0g) / R).astype(int), 0, nsg - 1), np.clip(((L - L0g) / R).astype(int), 0, nlg - 1)]
    new = ~cell_known & np.array([(int(np.floor(u_ / 0.1)), int(np.floor(v_ / 0.1))) not in occ for u_, v_ in zip(S, L)])
    rng = np.random.default_rng(0)
    S = np.repeat(S[new], per01) + rng.uniform(-0.05, 0.05, new.sum() * per01)
    L = np.repeat(L[new], per01) + rng.uniform(-0.05, 0.05, new.sum() * per01)
    k = np.clip(np.searchsorted(qs, S), 0, len(qs) - 1)
    fx = px[k] + gx[k] * (S - qs[k]) - gy[k] * L
    fy = py[k] + gy[k] * (S - qs[k]) + gx[k] * L
    fz = map_coordinates(Zg, [(S - S0g) / R - 0.5, (L - L0g) / R - 0.5], order=1, mode='nearest') + rng.choice(resid, len(S))
    outP = np.concatenate([outP, np.stack([fx, fy, fz], 1).astype(outP.dtype)])
    outI = np.concatenate([outI, rng.choice(inten, len(S)).astype(outI.dtype)])
    np.save(os.path.join(a.track_dir, f'filled_{a.tag_out}.npy'), np.stack([fx, fy, fz], 1))
    print(f'start hole: {(~known).sum()} of {known.size} 0.2 m cells solved (Laplace) from {known.sum()} real road cells; '
          f'{len(S)} synthetic points ({per01} per 0.1 m cell, residual sd {resid.std():.3f} m, intensity median {np.median(inten):.0f}) '
          f'for s < {a.fill_to} m, l -{a.fill_right}..+{a.fill_left} m')
np.save(os.path.join(a.track_dir, f'map_{a.tag_out}.npy'), outP)
np.save(os.path.join(a.track_dir, f'map_intensity_{a.tag_out}.npy'), outI)
print('saved', f'map_{a.tag_out}.npy')
if os.environ.get('ROOF_STATS'):
    lh = np.histogram(lp[above], bins=np.arange(-5, 5.01, 1))[0]
    zh = np.histogram((zp - zref)[above], bins=[0.5, 1, 1.5, 2, 2.5, 3, 4])[0]
    print('cleared points by lateral offset l (m, left +):', dict(zip(np.arange(-5, 5).tolist(), lh.tolist())))
    print('by height above road (m):', dict(zip(['.5-1', '1-1.5', '1.5-2', '2-2.5', '2.5-3', '3-4'], zh.tolist())))
    occ = np.bincount(np.clip(((sp[above] + a.back) / 2).astype(int), 0, None))
    print(f'2 m path cells with > 50 cleared points: {(occ > 50).sum()} of {len(occ)}')
