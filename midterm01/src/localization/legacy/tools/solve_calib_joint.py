#!/usr/bin/env python3
"""Joint calibration from hand-marked pairs: per-camera rotation, rig height, and per-frame pose nuisance.

The reference trajectory is only ~0.2 m accurate, and pairs of one frame share that frame's pose error (seen as
frame-consistent pixel offsets). Unknowns: camera height h, per-camera (yaw, pitch, roll) corrections, per-frame
(dx, dy, dyaw) of the vehicle; weak priors keep it well posed. Compares nested models by reprojection error.
"""
import json
import os
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
from camera_model import Cameras  # noqa: E402

work = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'work')
pairs = json.load(open(os.path.join(work, 'calib_pairs.json')))['pairs']
C = Cameras('/root/data/Track2/Track2_images/camera_info')
C.rot_fix = {}
Rm = Rotation.from_euler('ZYX', [-1.5, 0, 0], degrees=True).as_matrix()
cams = ['fl', 'f', 'fr']
frames = sorted({(p['track'], p['frame']) for p in pairs})
fidx = {f: k for k, f in enumerate(frames)}
X = np.array([p['xyz'] for p in pairs])          # vehicle frame at the reference pose, z = -1.5 (assumed height)
U = np.array([p['uv_img'] for p in pairs])
cid = np.array([cams.index(p['cam']) for p in pairs])
fid = np.array([fidx[(p['track'], p['frame'])] for p in pairs])
base = {c: (C.yaw_fix.get(c, 0.0), 0.0, 0.0) for c in cams}


def unpack(v, use_h, use_rot, use_frame):
    k = 0
    h = 1.5
    if use_h:
        h = v[k]; k += 1
    rot = {c: list(base[c]) for c in cams}
    if use_rot:
        for c in cams:
            rot[c] = list(v[k:k + 3]); k += 3
    fr = np.zeros((len(frames), 3))
    if use_frame:
        fr = v[k:].reshape(-1, 3)
    return h, rot, fr


def resid(v, use_h, use_rot, use_frame):
    h, rot, fr = unpack(v, use_h, use_rot, use_frame)
    out = []
    for c_i, c in enumerate(cams):
        m = cid == c_i
        if not m.any():
            continue
        C.rot_fix[c] = tuple(rot[c])
        P = X[m].copy()
        P[:, 2] = -h
        # frame pose nuisance: true vehicle pose = reference + (dx, dy, dyaw) -> point seen at R(-dyaw)(p - d)
        d = fr[fid[m]]
        ca, sa = np.cos(-np.radians(d[:, 2])), np.sin(-np.radians(d[:, 2]))
        px, py = P[:, 0] - d[:, 0], P[:, 1] - d[:, 1]
        P[:, 0], P[:, 1] = ca * px - sa * py, sa * px + ca * py
        uv, ok = C.project(c, P @ Rm)
        r = uv - U[m]
        r[~ok] = 100.0
        out.append(r.ravel())
    pri = []
    if use_rot:
        for c in cams:
            pri += list((np.array(rot[c]) - np.array(base[c])) / 3.0)      # rotation prior sigma 3 deg (in px-ish units)
    if use_frame:
        pri += list((fr[:, :2] / 0.3).ravel()) + list(fr[:, 2] / 0.5)     # 0.3 m, 0.5 deg
    if use_h:
        pri += [(h - 1.5) / 0.5]
    return np.concatenate(out + [np.array(pri) * 2.0])


def fit(use_h, use_rot, use_frame):
    v0 = ([1.5] if use_h else []) + (sum([list(base[c]) for c in cams], []) if use_rot else []) + ([0.0] * 3 * len(frames) if use_frame else [])
    sol = least_squares(resid, np.array(v0), args=(use_h, use_rot, use_frame), loss='huber', f_scale=6.0) if v0 else None
    v = sol.x if sol is not None else np.array([])
    r = resid(v, use_h, use_rot, use_frame)[:2 * len(pairs)]
    return v, np.sqrt(np.mean(r ** 2)), np.median(np.hypot(r[0::2], r[1::2]))


print(f'{len(pairs)} pairs, {len(frames)} frames')
for name, cfg in [('nothing (current calib baseline)', (False, False, False)), ('per-frame pose only', (False, False, True)),
                  ('camera rotations + per-frame pose', (False, True, True)), ('height + per-frame pose', (True, False, True)),
                  ('height + rotations + per-frame pose', (True, True, True))]:
    v, rms, med = fit(*cfg)
    h, rot, fr = unpack(v, *cfg)
    txt = f'{name:38s}: reprojection rms {rms:5.1f} px, median {med:4.1f} px'
    if cfg[0]:
        txt += f' | camera height {h:.2f} m'
    if cfg[1]:
        txt += ' | rot ' + ' '.join(f'{c}:({rot[c][0]:+.1f},{rot[c][1]:+.1f},{rot[c][2]:+.1f})' for c in cams)
    if cfg[2]:
        txt += ' | frame offsets ' + ' '.join(f'{f[1]}:({d[0]:+.2f},{d[1]:+.2f}m,{d[2]:+.1f}°)' for f, d in zip(frames, fr))
    print(txt)
