#!/usr/bin/env python3
"""Find where the map / camera fixes still disagree with a trajectory (review list for the 8765 viewer).

For each fix type (BEV lanes, road/kerb, poles) the residual fix - trajectory is expressed in the trajectory's
forward / lateral axes (and yaw for lane fixes) and summarised per window of frames. A window is flagged when the
fixes of one type are consistently off (median beyond a threshold with enough fixes) or when two types disagree with
each other (one of them, or the camera that sees it, is wrong). Output: JSON list for pole_review_server /issues.
"""
import argparse
import json

import numpy as np
import pandas as pd

ap = argparse.ArgumentParser()
ap.add_argument('traj_csv')
ap.add_argument('out_json')
ap.add_argument('--lane', required=True)
ap.add_argument('--road', required=True)
ap.add_argument('--pole', required=True)
ap.add_argument('--win', type=int, default=60)
ap.add_argument('--thr', type=float, default=0.15, help='m, flag a type whose median residual exceeds this')
ap.add_argument('--yaw_thr', type=float, default=0.4, help='deg, flag lane yaw residual beyond this')
a = ap.parse_args()
tr = pd.read_csv(a.traj_csv)
n = len(tr)
yaw = np.unwrap(tr.yaw.values)


def resid(f, pct):
    z = np.load(f)
    keep = z['peak'] >= np.percentile(z['peak'], pct)
    if 'dyaw' in z:  # drop fixes at the edge of the yaw search (no real peak)
        dd = np.abs(np.degrees(z['dyaw']))
        lim = z['max_yaw'] if 'max_yaw' in z else np.full(len(dd), dd.max())
        keep &= dd < lim - 0.05
    i = z['idx'][keep].astype(int)
    dx, dy = z['mx'][keep] - tr.x.values[i], z['my'][keep] - tr.y.values[i]
    c, s = np.cos(yaw[i]), np.sin(yaw[i])
    out = {'i': i, 'fwd': c * dx + s * dy, 'lat': -s * dx + c * dy}
    if 'myaw' in z:
        out['yaw'] = np.degrees(np.angle(np.exp(1j * (z['myaw'][keep] - yaw[i]))))
    return out


R = {'lane': resid(a.lane, 30), 'road': resid(a.road, 50), 'pole': resid(a.pole, 60)}
AX = {'lane': ['lat', 'yaw'], 'road': ['lat'], 'pole': ['fwd', 'lat']}   # axes each fix type measures well
issues = []
for lo in range(0, n, a.win):
    hi = min(lo + a.win, n)
    stats = {}
    for k, r in R.items():
        m = (r['i'] >= lo) & (r['i'] < hi)
        if m.sum() < 5:
            continue
        stats[k] = {ax: float(np.median(r[ax][m])) for ax in AX[k]}
        stats[k]['n'] = int(m.sum())
    flags = []
    for k, st in stats.items():
        for ax in AX[k]:
            v = st[ax]
            if (ax == 'yaw' and abs(v) > a.yaw_thr) or (ax != 'yaw' and abs(v) > a.thr):
                flags.append(f'{k} {ax} {v:+.2f}{"°" if ax == "yaw" else " m"}')
    if 'lane' in stats and 'road' in stats and abs(stats['lane']['lat'] - stats['road']['lat']) > a.thr:
        flags.append(f'lane vs road lateral differ {stats["lane"]["lat"] - stats["road"]["lat"]:+.2f} m')
    if 'lane' in stats and 'pole' in stats and abs(stats['lane']['lat'] - stats['pole']['lat']) > 2 * a.thr:
        flags.append(f'lane vs pole lateral differ {stats["lane"]["lat"] - stats["pole"]["lat"]:+.2f} m')
    if not flags:
        continue
    lat = [st['lat'] for st in stats.values() if 'lat' in st]
    same_side = len(lat) >= 2 and (all(v > a.thr * 0.6 for v in lat) or all(v < -a.thr * 0.6 for v in lat))
    if same_side:
        hint = 'all map fixes agree, trajectory sits off them: map distortion here, or the trajectory leans on GNSS'
    elif any('differ' in f for f in flags):
        hint = 'fix types disagree: one map layer (markings / kerb / poles) or the camera that sees it is off'
    elif any('yaw' in f for f in flags):
        hint = 'heading residual: trajectory yaw or camera rotation'
    else:
        hint = 'single fix type off: check that layer in the viewer'
    sev = max([abs(st[ax]) / (a.yaw_thr / a.thr if ax == 'yaw' else 1) for st in stats.values() for ax in st if ax != 'n'])
    issues.append({'lo': lo, 'hi': hi, 'frame': lo + (hi - lo) // 2, 'stats': stats, 'flags': flags, 'hint': hint,
                   'sev': round(float(sev), 3)})
json.dump({'traj': a.traj_csv, 'n': n, 'win': a.win, 'issues': issues}, open(a.out_json, 'w'), indent=1)
print(f'{len(issues)} flagged windows of {(n + a.win - 1) // a.win}')
for it in sorted(issues, key=lambda x: -x['sev'])[:12]:
    print(f"  {it['lo']:4d}-{it['hi']:4d} sev {it['sev']:.2f}: {'; '.join(it['flags'])}  -> {it['hint'][:40]}")
