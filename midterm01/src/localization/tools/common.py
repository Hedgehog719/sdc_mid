"""Shared helpers: frame index, sensor interpolation, map tiling, pose math."""
import os

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation, Slerp


def ply_frames(ply_dir):
    files = sorted(f for f in os.listdir(ply_dir) if f.endswith('.ply'))
    files.sort(key=lambda f: int(f.split('_ts_')[1][:-4]))
    ts = np.array([int(f.split('_ts_')[1][:-4]) for f in files])
    return ts, [os.path.join(ply_dir, f) for f in files]


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class Sensors:
    """GNSS + IMU interpolated at arbitrary ns timestamps (bag receive time)."""

    def __init__(self, track_dir):
        g = pd.read_csv(os.path.join(track_dir, 'sensors', 'gnss.csv'))
        im = pd.read_csv(os.path.join(track_dir, 'sensors', 'imu.csv'))
        self.gt = g.t.values.astype(np.float64)
        self.gxyz = g[['x', 'y', 'z']].values
        self.gyaw = np.unwrap(2 * np.arctan2(g.qz.values, g.qw.values))
        self.it = im.t.values.astype(np.float64)
        q = im[['qx', 'qy', 'qz', 'qw']].values
        self.irot = Rotation.from_quat(q)
        self.islerp = Slerp(self.it, self.irot)
        self.igyro = im[['wx', 'wy', 'wz']].values

    def gnss_xyz(self, t):
        t = np.atleast_1d(t).astype(np.float64)
        return np.stack([np.interp(t, self.gt, self.gxyz[:, i]) for i in range(3)], axis=1)

    def gnss_yaw(self, t):
        return np.interp(np.atleast_1d(t).astype(np.float64), self.gt, self.gyaw)

    def imu_rot(self, t):
        t = np.clip(np.atleast_1d(t).astype(np.float64), self.it[0], self.it[-1])
        return self.islerp(t)

    def prior_pose(self, t, yaw_offset=0.0):
        """4x4 prior: GNSS position, IMU roll/pitch, GNSS yaw (+offset)."""
        t = np.atleast_1d(t)
        xyz = self.gnss_xyz(t)
        eul = self.imu_rot(t).as_euler('ZYX')  # yaw, pitch, roll
        eul[:, 0] = self.gnss_yaw(t) + yaw_offset
        R = Rotation.from_euler('ZYX', eul).as_matrix()
        T = np.tile(np.eye(4), (len(t), 1, 1))
        T[:, :3, :3] = R
        T[:, :3, 3] = xyz
        return T


class TiledMap:
    """Full-resolution map bucketed in square XY tiles for fast radius queries."""

    def __init__(self, npy_path, tile=20.0):
        pts = np.load(npy_path, mmap_mode='r')
        self.tile = tile
        keys = np.floor(pts[:, :2] / tile).astype(np.int64)
        self.kx0, self.ky0 = keys.min(0)
        k = (keys[:, 0] - self.kx0) * 100000 + (keys[:, 1] - self.ky0)
        order = np.argsort(k, kind='stable')
        self.pts = np.ascontiguousarray(pts[order])
        ks = k[order]
        self.ukeys, self.starts = np.unique(ks, return_index=True)
        self.ends = np.append(self.starts[1:], len(ks))

    def query(self, x, y, radius):
        r = int(np.ceil(radius / self.tile))
        cx = int(np.floor(x / self.tile)) - self.kx0
        cy = int(np.floor(y / self.tile)) - self.ky0
        parts = []
        for ix in range(cx - r, cx + r + 1):
            for iy in range(cy - r, cy + r + 1):
                i = np.searchsorted(self.ukeys, ix * 100000 + iy)
                if i < len(self.ukeys) and self.ukeys[i] == ix * 100000 + iy:
                    parts.append(self.pts[self.starts[i]:self.ends[i]])
        if not parts:
            return np.empty((0, 3), np.float32)
        p = np.concatenate(parts)
        return p[(p[:, 0] - x) ** 2 + (p[:, 1] - y) ** 2 < radius * radius]


def yaw_of(T):
    return np.arctan2(T[..., 1, 0], T[..., 0, 0])
