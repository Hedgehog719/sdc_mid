"""Causal, deduplicated use of the supplied GNSS orientation as a weak heading prior."""
import numpy as np
from scipy.spatial.transform import Rotation


def quaternion_headings(quaternions):
    quaternions = np.asarray(quaternions, dtype=float)
    if quaternions.ndim != 2 or quaternions.shape[1] != 4:
        raise ValueError('expected an array of x,y,z,w quaternions')
    valid = np.isfinite(quaternions).all(axis=1) & (np.linalg.norm(quaternions, axis=1) > 1e-8)
    result = np.full(len(quaternions), np.nan)
    if valid.any():
        result[valid] = Rotation.from_quat(quaternions[valid]).as_euler('ZYX')[:, 0]
    return result


class HeadingPrior:
    def __init__(self, once=True, min_interval_ns=2_000_000_000, max_age_ns=500_000_000):
        if min_interval_ns < 0 or max_age_ns < 0:
            raise ValueError('time limits must be nonnegative')
        self.once = once
        self.min_interval_ns = int(min_interval_ns)
        self.max_age_ns = int(max_age_ns)
        self.last_time = None
        self.last_heading = None

    def consume(self, sample_time, current_time, heading):
        sample_time, current_time = int(sample_time), int(current_time)
        if sample_time > current_time or current_time - sample_time > self.max_age_ns or not np.isfinite(heading):
            return None
        if self.last_time is not None:
            if self.once or sample_time <= self.last_time or sample_time - self.last_time < self.min_interval_ns:
                return None
            difference = np.arctan2(np.sin(heading - self.last_heading), np.cos(heading - self.last_heading))
            if abs(difference) < 1e-8:
                return None
        self.last_time, self.last_heading = sample_time, float(heading)
        return float(heading)