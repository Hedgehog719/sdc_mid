#!/usr/bin/env python3
"""Initial pose for the GNSS-free pipeline (Track03): init_guess.txt refined by a wide lane match.

init_guess.txt is noisy (Track01: 0.3 m / 1 deg, Track02: 0.9 m / 18 deg off). The camera lane response of the first
frames is matched to the LiDAR marking map around it with bev_match (heading +-max_yaw, shift +-max_shift, no
accumulation since the prior is a single pose); the median of the frames' estimates is the initial pose.
Prints "x,y,yaw" (for track_seq.py --init) and writes it to <out_txt>.
usage: init_search.py <track_dir> <marks_S.npz> <out_txt> [--frames 0:15:3 --max_yaw 25 --max_shift 3.5]
"""
import argparse
import os
import subprocess
import sys

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
from common import ply_frames  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('marks')
ap.add_argument('out_txt')
ap.add_argument('--frames', default='0:15:3')
ap.add_argument('--max_yaw', type=float, default=25.0)
ap.add_argument('--max_shift', type=float, default=3.5)
ap.add_argument('--init', default='', help='x,y,yaw_rad instead of init_guess.txt')
ap.add_argument('--rounds', type=int, default=2, help='re-centre and search again with a smaller window')
a = ap.parse_args()

ts, _ = ply_frames(os.path.join(a.track_dir, 'raw_ply'))
n = len(ts)
if a.init:
    x0, y0, yaw0 = (float(v) for v in a.init.split(','))
else:
    M = np.loadtxt(os.path.join(a.track_dir, 'init_guess.txt'), delimiter=',')
    x0, y0, yaw0 = M[0, 3], M[1, 3], Rotation.from_matrix(M[:3, :3]).as_euler('ZYX')[0]
print(f'init_guess: x {x0:.2f} y {y0:.2f} yaw {np.degrees(yaw0):.2f} deg', flush=True)
tool = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'bev_match.py')
base = os.path.splitext(a.out_txt)[0]
win = [(a.max_yaw, a.max_shift), (3.0, 1.0)][:a.rounds]
for rnd, (my, ms) in enumerate(win):
    prior = base + f'_prior{rnd}.csv'
    pd.DataFrame({'idx': np.arange(n), 'x': np.full(n, x0), 'y': np.full(n, y0), 'yaw': np.full(n, yaw0)}).to_csv(prior, index=False)
    out = base + f'_bm{rnd}.npz'
    cmd = [sys.executable, tool, a.track_dir, prior, out, '--frames', a.frames, '--max_yaw', str(my), '--yaw_step',
           '1.0' if my > 5 else '0.5', '--max_shift', str(ms), '--percam', '--map_marks', a.marks, '--yaw_mount', '-1.5',
           '--ground_mode', 'profile', '--yaw_fine', '0.1', '--workers', '4']
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
    z = np.load(out)
    ok = np.ones(len(z['idx']), bool)
    if 'max_yaw' in z:
        ok &= np.abs(np.degrees(z['dyaw'])) < z['max_yaw'] - 0.05
    if ok.sum() == 0:
        ok[:] = True
    mx, myy, myaw = np.median(z['mx'][ok]), np.median(z['my'][ok]), np.median(np.unwrap(z['myaw'][ok]))
    print(f'round {rnd} (yaw +-{my}, shift +-{ms} m): {ok.sum()}/{len(ok)} frames, x {mx:.2f} y {myy:.2f} yaw '
          f'{np.degrees(myaw):.2f} deg (spread x {np.std(z["mx"][ok]):.2f} y {np.std(z["my"][ok]):.2f} yaw '
          f'{np.degrees(np.std(z["myaw"][ok])):.2f}), peak median {np.median(z["peak"][ok]):.3f}', flush=True)
    x0, y0, yaw0 = mx, myy, myaw
with open(a.out_txt, 'w') as f:
    f.write(f'{x0:.4f},{y0:.4f},{yaw0:.6f}\n')
print(f'{x0:.4f},{y0:.4f},{yaw0:.6f}')
