#!/usr/bin/env python3
"""Cut a slice of the map out for manual editing (CloudCompare) and splice the edited slice back in.

export: points within --radius m (horizontal) of the trajectory frames --frames are written as a binary PLY
        (x, y, z float32 relative to an offset, so CloudCompare keeps cm precision without its global shift;
        scalar field 'intensity'), plus <name>_traj.ply (the driven path of those frames, coloured, same offset)
        and <name>.json (source map tag, offset, the indices of the cut points).
import: reads the edited PLY (any vertex layout CloudCompare writes: x, y, z float/double; intensity under
        'intensity' / 'scalar_intensity' / 'scalar_Intensity', else 0), adds the offset back, drops the original
        slice points from the source map and appends the edited ones -> map_<tag_out>.npy + map_intensity_<tag_out>.npy.
        Delete, move, rotate (e.g. fix a roll) or add points freely in CloudCompare; keep the offset frame.

usage: pcd_slice.py export <track_dir> <traj_csv> <out_dir/name> [--tag clean --frames 0:400 --radius 40]
       pcd_slice.py import <track_dir> <edited.ply> <out_dir/name.json> --tag_out cc1
"""
import argparse
import json
import os

import numpy as np
import pandas as pd
from scipy.spatial import cKDTree


def write_ply(path, xyz, scalars=None, rgb=None):
    names = [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]
    if scalars:
        names += [(k, '<f4') for k in scalars]
    if rgb is not None:
        names += [('red', 'u1'), ('green', 'u1'), ('blue', 'u1')]
    v = np.empty(len(xyz), dtype=names)
    v['x'], v['y'], v['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    for k, s in (scalars or {}).items():
        v[k] = s
    if rgb is not None:
        v['red'], v['green'], v['blue'] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    head = ['ply', 'format binary_little_endian 1.0', f'element vertex {len(v)}']
    head += [f'property {"float" if t == "<f4" else "uchar"} {n}' for n, t in names] + ['end_header']
    with open(path, 'wb') as f:
        f.write(('\n'.join(head) + '\n').encode())
        f.write(v.tobytes())


PLY_T = {'char': 'i1', 'int8': 'i1', 'uchar': 'u1', 'uint8': 'u1', 'short': '<i2', 'int16': '<i2', 'ushort': '<u2',
         'uint16': '<u2', 'int': '<i4', 'int32': '<i4', 'uint': '<u4', 'uint32': '<u4', 'float': '<f4',
         'float32': '<f4', 'double': '<f8', 'float64': '<f8'}


def read_ply(path):
    """Vertex element of a PLY (binary little endian or ascii) as a structured array."""
    with open(path, 'rb') as f:
        fmt, elems, cur = None, [], None
        while True:
            ln = f.readline().decode('ascii', 'replace').strip()
            w = ln.split()
            if not w:
                continue
            if w[0] == 'format':
                fmt = w[1]
            elif w[0] == 'element':
                cur = [w[1], int(w[2]), []]
                elems.append(cur)
            elif w[0] == 'property':
                if w[1] == 'list':
                    raise SystemExit(f'list property in element {cur[0]} not supported (save the cloud without faces)')
                cur[2].append((w[2], PLY_T[w[1]]))
            elif w[0] == 'end_header':
                break
        if elems[0][0] != 'vertex':
            raise SystemExit('first PLY element is not vertex')
        n, props = elems[0][1], elems[0][2]
        if fmt == 'binary_little_endian':
            return np.frombuffer(f.read(n * np.dtype(props).itemsize), dtype=props, count=n)
        if fmt == 'ascii':
            a = np.loadtxt(f, max_rows=n, ndmin=2)
            out = np.empty(n, dtype=props)
            for k, (nm, _) in enumerate(props):
                out[nm] = a[:, k]
            return out
        raise SystemExit(f'PLY format {fmt} not supported (save as binary little endian or ascii)')


ap = argparse.ArgumentParser()
sub = ap.add_subparsers(dest='cmd', required=True)
e = sub.add_parser('export')
e.add_argument('track_dir')
e.add_argument('traj_csv')
e.add_argument('out')
e.add_argument('--tag', default='clean')
e.add_argument('--frames', default='0:400')
e.add_argument('--radius', type=float, default=40.0)
i = sub.add_parser('import')
i.add_argument('track_dir')
i.add_argument('ply')
i.add_argument('meta')
i.add_argument('--tag_out', required=True)
a = ap.parse_args()

if a.cmd == 'export':
    sfx = f'_{a.tag}' if a.tag else ''
    P = np.load(os.path.join(a.track_dir, f'map{sfx}.npy'), mmap_mode='r')
    I = np.load(os.path.join(a.track_dir, f'map_intensity{sfx}.npy'), mmap_mode='r')
    tr = pd.read_csv(a.traj_csv)
    f0, f1 = (int(v) for v in a.frames.split(':'))
    path = tr[['x', 'y']].values[f0:f1]
    d, _ = cKDTree(path).query(np.asarray(P[:, :2]), distance_upper_bound=a.radius)
    idx = np.flatnonzero(np.isfinite(d))
    off = np.round(path.mean(0)).tolist() + [0.0]
    xyz = np.asarray(P[idx], dtype=np.float64) - off
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    write_ply(a.out + '.ply', xyz.astype(np.float32), {'intensity': np.asarray(I[idx], dtype=np.float32)})
    # driven path: the slice frames as a dense dotted line at the map ground (+ start in green, end in red)
    gz = np.median(xyz[:, 2]) if len(xyz) else 0.0
    tree = cKDTree(xyz[:, :2])
    pz = []
    for px, py in path - off[:2]:
        nb = tree.query_ball_point((px, py), 1.5)
        pz.append(np.percentile(xyz[nb, 2], 10) if len(nb) > 5 else gz)
    T = np.c_[path - off[:2], pz]
    col = np.tile([255, 255, 0], (len(T), 1)).astype(np.uint8)
    col[:10] = (0, 255, 0)
    col[-10:] = (255, 0, 0)
    write_ply(a.out + '_traj.ply', T.astype(np.float32), rgb=col)
    json.dump({'track_dir': a.track_dir, 'tag': a.tag, 'offset': off, 'frames': a.frames, 'radius': a.radius,
               'n': int(len(idx))}, open(a.out + '.json', 'w'), indent=1)
    np.save(a.out + '_idx.npy', idx)
    print(f'exported {len(idx)} points of map{sfx} within {a.radius} m of frames {a.frames} -> {a.out}.ply '
          f'(offset {off}), path -> {a.out}_traj.ply')
else:
    m = json.load(open(a.meta))
    idx = np.load(a.meta[:-5] + '_idx.npy')
    sfx = f'_{m["tag"]}' if m['tag'] else ''
    P = np.load(os.path.join(a.track_dir, f'map{sfx}.npy'))
    I = np.load(os.path.join(a.track_dir, f'map_intensity{sfx}.npy'))
    v = read_ply(a.ply)
    xyz = np.stack([v['x'], v['y'], v['z']], 1).astype(np.float64) + m['offset']
    ik = next((k for k in ('intensity', 'scalar_intensity', 'scalar_Intensity', 'Intensity') if k in v.dtype.names), None)
    inten = v[ik].astype(np.float32) if ik else np.zeros(len(v), np.float32)
    keep = np.ones(len(P), bool)
    keep[idx] = False
    out = np.concatenate([P[keep], xyz.astype(P.dtype)])
    outi = np.concatenate([I[keep], inten.astype(I.dtype)])
    np.save(os.path.join(a.track_dir, f'map_{a.tag_out}.npy'), out)
    np.save(os.path.join(a.track_dir, f'map_intensity_{a.tag_out}.npy'), outi)
    print(f'slice {len(idx)} -> {len(v)} points (intensity field: {ik}); map_{a.tag_out}.npy {len(out)} points')
