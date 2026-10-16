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

    def __init__(self, track_dir, regularize_gnss=None):
        g = pd.read_csv(os.path.join(track_dir, 'sensors', 'gnss.csv'))
        im = pd.read_csv(os.path.join(track_dir, 'sensors', 'imu.csv'))
        self.gt = g.t.values.astype(np.float64)
        if regularize_gnss is None:
            regularize_gnss = os.environ.get('GNSS_REGULAR', '0') == '1'
        if regularize_gnss:
            # GNSS fixes are sampled on a regular 4 Hz grid; only the stamps jitter (step length is
            # uncorrelated with stamp dt). Re-time them with a robust linear fit over the slot index.
            period = np.median(np.diff(self.gt))
            slot = np.concatenate([[0], np.cumsum(np.round(np.diff(self.gt) / period))])
            A = np.stack([slot, np.ones_like(slot)], 1)
            keep = np.ones(len(slot), bool)
            for _ in range(5):
                coef = np.linalg.lstsq(A[keep], self.gt[keep], rcond=None)[0]
                r = self.gt - A @ coef
                keep = np.abs(r) < 3 * max(np.median(np.abs(r[keep])) * 1.4826, 1e5)
            self.gt = A @ coef
        self.gxyz = g[['x', 'y', 'z']].values
        self.gyaw = np.unwrap(2 * np.arctan2(g.qz.values, g.qw.values))
        self.it = im.t.values.astype(np.float64)
        q = im[['qx', 'qy', 'qz', 'qw']].values
        self.irot = Rotation.from_quat(q)
        self.islerp = Slerp(self.it, self.irot)
        self.igyro = im[['wx', 'wy', 'wz']].values

    def gnss_xyz(self, t, method='linear'):
        t = np.atleast_1d(t).astype(np.float64)
        if method == 'cubic':
            from scipy.interpolate import CubicSpline
            cs = CubicSpline((self.gt - self.gt[0]) * 1e-9, self.gxyz, axis=0, extrapolate=True)
            return cs((t - self.gt[0]) * 1e-9)
        return np.stack([np.interp(t, self.gt, self.gxyz[:, i]) for i in range(3)], axis=1)

    def imu_yaw_cf(self, t, sigma_s, ref='gnss'):
        """Complementary filter: IMU yaw + low-passed (GNSS - IMU) yaw offset (Gaussian sigma in seconds)."""
        from scipy.ndimage import gaussian_filter1d
        imu = np.unwrap(self.imu_rot(self.gt).as_euler('ZYX')[:, 0])
        off = np.unwrap(self.gyaw - imu)
        dt = np.median(np.diff(self.gt)) * 1e-9
        if ref == 'motion':  # heading of travel from GNSS positions, only while moving
            d = np.gradient(self.gxyz[:, :2], axis=0)
            moving = np.linalg.norm(d, axis=1) > 0.5  # > 2 m/s at 4 Hz
            mot = np.arctan2(d[:, 1], d[:, 0])
            off = np.unwrap(wrap(mot - imu))
            w = moving.astype(float)
            off_s = gaussian_filter1d(off * w, sigma_s / dt, mode='nearest') / np.maximum(
                gaussian_filter1d(w, sigma_s / dt, mode='nearest'), 1e-6)
        else:
            off_s = gaussian_filter1d(off, sigma_s / dt, mode='nearest')
        t = np.atleast_1d(t).astype(np.float64)
        return np.unwrap(self.imu_rot(t).as_euler('ZYX')[:, 0]) + np.interp(t, self.gt, off_s)

    def gnss_yaw(self, t):
        return np.interp(np.atleast_1d(t).astype(np.float64), self.gt, self.gyaw)

    def imu_rot(self, t):
        t = np.clip(np.atleast_1d(t).astype(np.float64), self.it[0], self.it[-1])
        return self.islerp(t)

    def imu_yaw(self, t):
        """IMU yaw shifted by its median offset to GNSS yaw (IMU heading is smoother, GNSS is unbiased)."""
        if not hasattr(self, '_imu_yaw_offset'):
            ref = np.unwrap(self.imu_rot(self.gt).as_euler('ZYX')[:, 0])
            self._imu_yaw_offset = np.median(wrap(self.gyaw - ref))
        return np.unwrap(self.imu_rot(t).as_euler('ZYX')[:, 0]) + self._imu_yaw_offset

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
