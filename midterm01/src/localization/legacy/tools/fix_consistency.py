#!/usr/bin/env python3
"""Temporal-consistency gate for map fixes: express every fix as (fwd, lat) offset from a reference trajectory
(e.g. the first fusion pass); consecutive fixes describe nearly the same offset (the reference's own error changes
slowly), so a fix whose offset differs from the median of its neighbours (+-win frames) by more than the gate is a
mismatch (periodic markings alias by one dash / stripe) and is dropped.
usage: fix_consistency.py fixes.npz ref.csv out.npz [--win 15 --gate_fwd 0.5 --gate_lat 0.3]"""
import argparse
import numpy as np
import pandas as pd
ap = argparse.ArgumentParser()
ap.add_argument('fixes'); ap.add_argument('ref'); ap.add_argument('out')
ap.add_argument('--win', type=int, default=15)
ap.add_argument('--gate_fwd', type=float, default=0.5)
ap.add_argument('--gate_lat', type=float, default=0.3)
a = ap.parse_args()
z = dict(np.load(a.fixes)); T = pd.read_csv(a.ref)
i = z['idx'].astype(int); yw = T.yaw.values[i]
dx, dy = z['mx'] - T.x.values[i], z['my'] - T.y.values[i]
fw, lt = np.cos(yw) * dx + np.sin(yw) * dy, -np.sin(yw) * dx + np.cos(yw) * dy
keep = np.ones(len(i), bool)
for k in range(len(i)):
    nb = (np.abs(i - i[k]) <= a.win) & (np.arange(len(i)) != k)
    if nb.sum() < 3: continue
    keep[k] = (abs(fw[k] - np.median(fw[nb])) <= a.gate_fwd) and (abs(lt[k] - np.median(lt[nb])) <= a.gate_lat)
np.savez(a.out, **{kk: v[keep] for kk, v in z.items()})
print(f'kept {keep.sum()} / {len(keep)}; dropped at frames {i[~keep][:40].tolist()}{" ..." if (~keep).sum() > 40 else ""}')
