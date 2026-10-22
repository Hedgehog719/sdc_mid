#!/usr/bin/env python3
"""Audit the LiDAR map along the driven path: where can it mislead the matchers?

Every --step metres along the trajectory, in a 5 m x 14 m patch of map points around the vehicle (u along, v left):
  layer      double ground layer: per 0.25 m road cell the z spread (p90 - p10) of the near-ground points; a single
             clean layer is a few cm, misregistered / pitched scans give 0.1-0.2 m
  vert       vertical distortion: map road height minus GNSS altitude, high-passed (< 60 m wavelengths); the real
             street has no such bumps if the map is right (GNSS altitude is smooth over seconds)
  roof       share of driven-lane points 1.3-2.2 m above the road: the mapping car's own roof, scanned along its path
  objects    map points 0.3-1.2 m above the road inside the driven lane (|v| < 1 m): our car drove through there,
             so these are vehicles / people baked into the map or noise
  coverage   fraction of road cells (|v| < 5 m) without ground points (holes)
  intensity  road background intensity jump between neighbouring stations (scans with different calibration)
Stations whose metric is far outside the track's robust spread (and beyond an absolute floor) form segments.
Output JSON for pole_review_server /audit/<Track>.
"""
import argparse
import json
import os
import sys

import numpy as np
import pandas as pd
from scipy.ndimage import gaussian_filter1d

sys.path.insert(0, os.path.dirname(__file__))
from bev_match import load_map  # noqa: E402
from common import Sensors, ply_frames  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('traj_csv')
ap.add_argument('out_json')
ap.add_argument('--step', type=float, default=2.0)
a = ap.parse_args()

tr = pd.read_csv(a.traj_csv)
ts, _ = ply_frames(os.path.join(a.track_dir, 'raw_ply'))
S = Sensors(a.track_dir)
M = load_map(a.track_dir)
s_cum = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(tr.x.values), np.diff(tr.y.values)))])
targets = np.arange(0, s_cum[-1], a.step)
frames = np.unique(np.searchsorted(s_cum, targets).clip(0, len(tr) - 1))
CELL = 0.25
rows = []
for i in frames:
    x, y, yaw = tr.x[i], tr.y[i], tr.yaw[i]
    idx = M.query_idx(x, y, 8.0)
    p = M.pts[idx]
    it = np.asarray(M.inten[idx], np.float32)
    c, s = np.cos(yaw), np.sin(yaw)
    d = p[:, :2] - [x, y]
    u, v = c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]
    m = (np.abs(u) < 2.5) & (np.abs(v) < 7)
    u, v, z, it = u[m], v[m], p[m, 2].astype(np.float64), it[m]
    if len(z) < 500:
        rows.append([i, s_cum[i]] + [np.nan] * 10)
        continue
    cu, cv = np.floor((u + 2.5) / CELL).astype(int), np.floor((v + 7) / CELL).astype(int)
    nv = int(14 / CELL) + 1
    key = cu * nv + cv
    zmin = np.full(key.max() + 1, np.inf)
    np.minimum.at(zmin, key, z)
    zc = zmin[key]
    near = z - zc < 0.4
    road = np.abs(v) < 4
    # plane of the road under the vehicle from the cell minima
    uk = np.unique(key[road])
    ku, kv = uk // nv, uk % nv
    cu_c, cv_c = ku * CELL - 2.5 + CELL / 2, kv * CELL - 7 + CELL / 2
    zz = zmin[uk]
    keep = np.ones(len(zz), bool)
    for _ in range(3):
        A = np.stack([np.ones(keep.sum()), cu_c[keep], cv_c[keep]], 1)
        coef = np.linalg.lstsq(A, zz[keep], rcond=None)[0]
        r = zz - (coef[0] + coef[1] * cu_c + coef[2] * cv_c)
        keep = np.abs(r) < max(3 * 1.4826 * np.median(np.abs(r[keep])), 0.05)
    z_road = coef[0]
    hrel = z - (coef[0] + coef[1] * u + coef[2] * v)
    # layer spread per road cell
    mk = near & road
    kk, zs = key[mk], z[mk]
    o = np.lexsort((zs, kk))
    kk, zs = kk[o], zs[o]
    _, first, cnt = np.unique(kk, return_index=True, return_counts=True)
    g6 = cnt >= 6
    sp = zs[first[g6] + (0.9 * (cnt[g6] - 1)).astype(int)] - zs[first[g6] + (0.1 * (cnt[g6] - 1)).astype(int)]
    layer = float(np.median(sp)) if len(sp) else np.nan
    dbl = float(np.mean(sp > 0.08)) if len(sp) else np.nan
    lane = (np.abs(v) < 1.0) & (np.abs(u) < 2.0)
    objects = int(((hrel > 0.3) & (hrel < 1.2) & lane).sum())   # below the mapping car's roof layer
    roof = float(((hrel > 1.3) & (hrel < 2.2) & lane).sum() / max(lane.sum(), 1))
    ground = np.abs(hrel) < 0.25
    gcells = np.unique(key[ground & (np.abs(v) < 5)])
    allc = (int(5 / CELL) * 2) * int(5 / CELL)
    holes = 1.0 - len(gcells) / allc
    inten = float(np.median(it[ground & road])) if (ground & road).sum() > 50 else np.nan
    rows.append([i, s_cum[i], z_road, np.degrees(np.arctan(coef[1])), np.degrees(np.arctan(coef[2])), layer, dbl,
                 objects, holes, inten, len(z), roof])
cols = ['frame', 's', 'z_road', 'grade', 'crossfall', 'layer', 'dbl', 'objects', 'holes', 'inten', 'npts', 'roof']
D = pd.DataFrame([r[:len(cols)] if len(r) >= len(cols) else r + [np.nan] * (len(cols) - len(r)) for r in rows], columns=cols)
fr = D.frame.values.astype(int)
# vertical distortion vs GNSS altitude (high-pass), grade vs IMU body pitch (low-pass, offset removed)
zg = S.gnss_xyz(ts[fr])[:, 2]
dz = D.z_road.values - zg
ok = np.isfinite(dz)
hp_sig = 60.0 / a.step / 2
dz_f = np.interp(np.arange(len(dz)), np.flatnonzero(ok), dz[ok])
D['vert'] = dz_f - gaussian_filter1d(dz_f, hp_sig, mode='nearest')
D['inten_jump'] = np.abs(np.gradient(gaussian_filter1d(np.nan_to_num(D.inten.values, nan=np.nanmedian(D.inten)), 1)))
# flags
spec = {  # metric: (absolute floor, label, hint)
    'layer': (0.06, 'double ground layer', 'road points spread vertically: two scans disagree here (misregistration or pitched scans); '
              'BEV road masks, kerb fixes and the 3D ground get noisy'),
    'vert': (0.10, 'vertical distortion', 'map road height has a bump / dip that the GNSS altitude does not show: map warped here; '
             'flat-BEV vs map geometry and the 3D ground disagree (far markings land too near / far)'),
    'objects': (30, 'objects in the driven lane', 'map points 0.3-1.2 m above the road where our car drove: vehicles / people '
                'captured while mapping, or noise; can create false kerbs / road edges / obstacles'),
    'holes': (0.35, 'coverage holes', 'many road cells without map points: markings / road edges missing in the map here'),
    'inten_jump': (1.0, 'intensity jump', 'road background intensity changes abruptly: scans with different intensity '
                   'calibration meet; marking contrast changes'),
}
segs = []
for k, (floor, label, hint) in spec.items():
    val = np.abs(D[k].values.astype(float))
    med = np.nanmedian(val)
    mad = 1.4826 * np.nanmedian(np.abs(val - med)) + 1e-9
    thr = max(floor, med + 4 * mad)
    flag = np.nan_to_num(val, nan=0) > thr
    j = 0
    while j < len(flag):
        if not flag[j]:
            j += 1
            continue
        e = j
        while e + 1 < len(flag) and (flag[e + 1] or (e + 2 < len(flag) and flag[e + 2])):
            e += 1
        seg = slice(j, e + 1)
        segs.append({'type': label, 'metric': k, 'lo': int(fr[j]), 'hi': int(fr[e]), 's_lo': round(float(D.s.values[j]), 1),
                     's_hi': round(float(D.s.values[e]), 1), 'peak': round(float(np.nanmax(val[seg])), 3),
                     'thr': round(float(thr), 3), 'sev': round(float(np.nanmax(val[seg]) / thr), 2), 'hint': hint,
                     'frame': int(fr[j + int(np.nanargmax(val[seg]))])})
        j = e + 1
segs.sort(key=lambda q: -q['sev'])
out = {'track': os.path.basename(os.path.normpath(a.track_dir)), 'step': a.step,
       'roof_frac': float(np.nanmedian(D.roof)), 'roof_height': None,
       'stations': {c: [None if not np.isfinite(v) else round(float(v), 4) for v in D[c].values] for c in
                    ['frame', 's', 'layer', 'vert', 'objects', 'holes', 'inten', 'z_road', 'roof']},
       'segments': segs}
json.dump(out, open(a.out_json, 'w'))
print(f"{out['track']}: {len(D)} stations over {s_cum[-1]:.0f} m; mapping-car roof layer (1.3-2.2 m above the road inside the driven lane): median {out['roof_frac'] * 100:.0f} % of lane points")
for k in spec:
    v = D[k].values.astype(float)
    print(f'  {k:10s} median {np.nanmedian(np.abs(v)):.3f}  p95 {np.nanpercentile(np.abs(v), 95):.3f}  max {np.nanmax(np.abs(v)):.3f}')
for q in segs[:20]:
    print(f"  [{q['sev']:.1f}] frames {q['lo']}-{q['hi']} ({q['s_lo']:.0f}-{q['s_hi']:.0f} m) {q['type']}: peak {q['peak']} (thr {q['thr']})")
