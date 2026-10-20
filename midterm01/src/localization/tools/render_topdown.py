#!/usr/bin/env python3
"""Top-down comparison video: LiDAR map + camera point cloud (projected to the ground) + trajectories.

Vehicle-centred, heading up. Map: ground points grey by intensity, raised points coloured by height above the local
ground. Camera cloud (3cameras_ply, front-camera FLU, unknown scale): scaled so its ground lies 1.70 m below the
camera, flattened onto the ground and drawn in its own colours over the map. Trajectory A (green, the one the view
follows) and B (red, optional) over +-N frames; the text gives B - A (forward / lateral / yaw) at the frame.
Writes an mp4 and viewer frames (work/viewer/<Track>_<name>/kf_00000.jpg + frames.json, 8765 viewer slider).

usage: render_topdown.py <track_dir> <traj_A.csv> [--b traj_B.csv] --name topdown --frames 0:-1:3 [--tag clean16]
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(__file__))
from common import TiledMap  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('traj_a')
ap.add_argument('--b', default='')
ap.add_argument('--name', default='topdown')
ap.add_argument('--frames', default='0:-1:3')
ap.add_argument('--tag', default='')
ap.add_argument('--res', type=float, default=0.08, help='m per pixel')
ap.add_argument('--size', type=int, default=600, help='panel size (three panels side by side)')
ap.add_argument('--cam_h', type=float, default=1.70)
ap.add_argument('--fps', type=float, default=10.0)
ap.add_argument('--alpha', type=float, default=0.55, help='camera cloud opacity')
ap.add_argument('--follow', choices=['a', 'b'], default='a', help='trajectory the view and the camera cloud follow')
ap.add_argument('--label_a', default='A')
ap.add_argument('--layout', choices=['triple', 'dual'], default='triple',
                help='triple: map | camera | overlay for one view; dual: overlay at A (left) | overlay at B (right)')
ap.add_argument('--label_b', default='B')
a = ap.parse_args()

T = os.path.basename(os.path.normpath(a.track_dir))
A = pd.read_csv(a.traj_a)
B = pd.read_csv(a.b) if a.b else None
sfx = f'_{a.tag}' if a.tag else ''
M = TiledMap(os.path.join(a.track_dir, f'map{sfx}.npy'), keep_order=True)
I = np.load(os.path.join(a.track_dir, f'map_intensity{sfx}.npy'), mmap_mode='r')
M.inten = np.minimum(np.asarray(I)[M.order], 255).astype(np.float32)
del M.order
idx = pd.read_csv(os.path.join(a.track_dir, '3cameras_ply', 'index.csv'))
S, R = a.size, a.res
row0 = int(S * 0.62)                       # vehicle row (more view ahead)


def read_ply(p):
    with open(p, 'rb') as f:
        n = None
        while True:
            ln = f.readline().decode().strip()
            if ln.startswith('element vertex'):
                n = int(ln.split()[-1])
            if ln == 'end_header':
                break
        d = np.frombuffer(f.read(n * 15), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
    return np.stack([d['x'], d['y'], d['z']], 1).astype(np.float64), np.stack([d['b'], d['g'], d['r']], 1)


def to_px(fw, lt):
    return (S / 2 - lt / R).astype(int), (row0 - fw / R).astype(int)


def local(x, y, px, py, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    dx, dy = x - px, y - py
    return c * dx + s * dy, -s * dx + c * dy


lo, hi, st = (int(v) for v in a.frames.split(':'))
frames = list(range(lo, len(A) if hi < 0 else hi, st))
out_dir = os.path.join(os.path.dirname(__file__), '..', 'work', 'viewer', f'{T}_{a.name}')
os.makedirs(out_dir, exist_ok=True)
mp4 = os.path.join(os.path.dirname(__file__), '..', 'work', 'video', f'{T}_{a.name}.mp4')
os.makedirs(os.path.dirname(mp4), exist_ok=True)
rad = S * R * 0.75


def view(fr, F):
    """Panels (map, camera cloud, overlay) centred on trajectory F at frame fr, both trajectories drawn."""
    px, py, yaw = F.x[fr], F.y[fr], F.yaw[fr]
    img = np.zeros((S, S, 3), np.uint8)
    q = M.query_idx(px, py, rad)
    P, inten = M.pts[q].astype(np.float64), M.inten[q]
    fw, lt = local(P[:, 0], P[:, 1], px, py, yaw)
    k = (np.floor(fw).astype(np.int64) + 1000) * 4000 + (np.floor(lt).astype(np.int64) + 1000)   # 1 m cells
    o = np.lexsort((P[:, 2], k))
    u, first, cnt = np.unique(k[o], return_index=True, return_counts=True)
    gz = np.empty(len(P))
    gz[o] = np.repeat(P[o][:, 2][first + (0.1 * (cnt - 1)).astype(int)], cnt)
    hz = P[:, 2] - gz
    cu, cv = to_px(fw, lt)
    ok = (cu >= 0) & (cu < S) & (cv >= 0) & (cv < S)
    gnd = ok & (hz < 0.25)
    g = np.clip(40 + inten[gnd] * 6, 0, 255).astype(np.uint8)
    img[cv[gnd], cu[gnd]] = np.stack([g, g, g], 1)
    hi_ = ok & (hz >= 0.25) & (hz < 6)
    col = cv2.applyColorMap(np.clip(hz[hi_] / 6 * 255, 0, 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(-1, 3)
    img[cv[hi_], cu[hi_]] = (col * 0.7).astype(np.uint8)
    cam = np.zeros((S, S, 3), np.uint8)
    ovl = (img.astype(np.float32) * 0.8).astype(np.uint8)
    sc = None
    if fr < len(idx):
        Q, rgb = read_ply(os.path.join(a.track_dir, '3cameras_ply', idx.ply[fr]))
        r = np.hypot(Q[:, 0], Q[:, 1])
        near = (Q[:, 0] > 3) & (Q[:, 0] < 15) & (np.abs(Q[:, 1]) < 3)
        if near.sum() > 100:
            G = Q[near]
            z0 = np.percentile(G[:, 2], 20)
            gg = G[np.abs(G[:, 2] - z0) < 0.25]
            pa, pb, pc = np.linalg.lstsq(np.c_[gg[:, 0], gg[:, 1], np.ones(len(gg))], gg[:, 2], rcond=None)[0]
            sc = a.cam_h / max(-pc, 0.5)
            m = (r > 1.5) & (r < 40 / sc)
            qu, qv = to_px(Q[m, 0] * sc, Q[m, 1] * sc)
            okc = (qu >= 1) & (qu < S - 1) & (qv >= 1) & (qv < S - 1)
            qu, qv, cc = qu[okc], qv[okc], rgb[m][okc]
            hq = ((Q[m, 2] - (pa * Q[m, 0] + pb * Q[m, 1] + pc)) * sc)[okc]
            o2 = np.argsort(hq)                      # raised points drawn last (on top)
            for du, dv in ((0, 0), (1, 0), (0, 1), (1, 1)):
                cam[qv[o2] + dv, qu[o2] + du] = cc[o2]
            lum = cc.astype(np.float32).mean(1)
            mk = (hq < 0.3) & (lum > np.percentile(lum, 85))     # bright ground = markings
            rs = hq >= 0.3                                       # raised structure
            ovl[qv[rs], qu[rs]] = (255, 255, 0)
            ovl[qv[mk], qu[mk]] = (255, 0, 255)
    panels = [img, cam, ovl]
    win = np.arange(max(fr - 150, 0), min(fr + 300, len(A)))
    for D, colr in ((A, (60, 220, 60)), (B, (60, 60, 255))):
        if D is None:
            continue
        f2, l2 = local(D.x.values[win], D.y.values[win], px, py, yaw)
        uu, vv = to_px(f2, l2)
        f0, l0 = local(D.x[fr], D.y[fr], px, py, yaw)
        u0, v0 = to_px(np.array([f0]), np.array([l0]))
        h = D.yaw[fr] - yaw
        for pn in panels:
            cv2.polylines(pn, [np.stack([uu, vv], 1).astype(np.int32)], False, colr, 1)
            cv2.arrowedLine(pn, (int(u0[0]), int(v0[0])), (int(u0[0] - 25 * np.sin(h)), int(v0[0] - 25 * np.cos(h))), colr, 2, tipLength=0.3)
    for pn in panels:
        cv2.line(pn, (S - 80, S - 30), (S - 80 + int(5 / R), S - 30), (200, 200, 200), 2)
        cv2.putText(pn, '5 m', (S - 70, S - 38), 0, 0.4, (200, 200, 200), 1)
    return panels, sc


dual = a.layout == 'dual' and B is not None
W = 2 * S if dual else 3 * S
vw = cv2.VideoWriter(mp4, cv2.VideoWriter_fourcc(*'mp4v'), a.fps, (W, S))
for fr in frames:
    txt = f'{T} frame {fr} | green {a.label_a}, red {a.label_b}'
    if B is not None:
        f0, l0 = local(B.x[fr], B.y[fr], A.x[fr], A.y[fr], A.yaw[fr])
        dyaw = np.degrees((B.yaw[fr] - A.yaw[fr] + np.pi) % (2 * np.pi) - np.pi)
        txt += f' | red - green: fwd {f0:+.2f} lat {l0:+.2f} m yaw {dyaw:+.2f} deg'
    if dual:
        pa_, sca = view(fr, A)
        pb_, scb = view(fr, B)
        left, right = pa_[2], pb_[2]
        cv2.putText(left, f'camera cloud placed at {a.label_a} (green)', (8, S - 12), 0, 0.5, (60, 220, 60), 1)
        cv2.putText(right, f'camera cloud placed at {a.label_b} (red)', (8, S - 12), 0, 0.5, (90, 90, 255), 1)
        img = np.hstack([left, right])
        cv2.line(img, (S, 0), (S, S), (255, 255, 255), 1)
        txt += ' | magenta: cam markings, cyan: cam raised' + (f' | scale {sca:.2f}' if sca else '')
    else:
        F = B if (a.follow == 'b' and B is not None) else A
        panels, sc = view(fr, F)
        for pn, ttl in zip(panels, ('LiDAR map', 'camera cloud on the ground' + (f' (scale {sc:.2f})' if sc else ''), 'overlay: magenta = camera markings, cyan = camera raised')):
            cv2.putText(pn, ttl, (8, S - 12), 0, 0.45, (220, 220, 220), 1)
        img = np.hstack(panels)
        txt += ' | view follows ' + (a.label_b if F is B else a.label_a)
    cv2.rectangle(img, (0, 0), (W, 30), (0, 0, 0), -1)
    cv2.putText(img, txt, (10, 21), 0, 0.5, (255, 255, 255), 1)
    vw.write(img)
    cv2.imwrite(os.path.join(out_dir, f'kf_{fr:05d}.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 88])
    cv2.imwrite(os.path.join(out_dir, f'bev_{fr:05d}.jpg'), img[:10])
vw.release()
json.dump({'track': T, 'frames': frames, 'label': f'{T} top-down: map + camera cloud on the ground; green {a.label_a}'
           + (f', red {a.label_b}' if a.b else '') + ('; left: camera at green, right: camera at red' if a.layout == 'dual' else f'; view follows {a.label_b if a.follow == "b" else a.label_a}')},
          open(os.path.join(out_dir, 'frames.json'), 'w'))
print('wrote', mp4, 'and', out_dir, len(frames), 'frames')
