"""Single-frame lane matching (bev_match.match_frame without accumulation) as a library: for sequential use without
a prior trajectory (Track03 tracker). setup() loads the map / marking map / camera BEV once; match(i, pose) returns
(peak, sharp, fwd, lat, dyaw, sharp_f, sharp_l) = the correction of pose (vehicle frame) or None."""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import bev as _bev  # noqa: E402
import bev_match as BM  # noqa: E402
from common import ply_frames  # noqa: E402
from marking_map import MarkingMap  # noqa: E402


def setup(track_dir, marks_npz, max_shift=2.5, max_yaw=3.0, yaw_step=0.5, yaw_fine=0.1, yaw_mount=-1.5):
    BM.A = argparse.Namespace(
        track_dir=track_dir, cam_height=1.5, pitch=0.0, roll=0.0, yaw_mount=yaw_mount, max_shift=max_shift,
        max_yaw=max_yaw, yaw_step=yaw_step, yaw_fine=yaw_fine, sign=-1.0, yaw_sign=-1.0, feature='lane',
        road_map='smooth', road_v3=False, road_seg='road', road_colour=False, curb_band=3.0, range_weight=0.0,
        map_marks=marks_npz, near_only=0.0, map_min_blob=0, pin_z=None, pin_frames='', hybrid_ratio=0.6,
        ground_mode='profile', color=False, percam=True, photonorm=False, accum=0, accum_step=4, accum_dist=0.0,
        accum_ds=0.7, ground3d=False, dyn_attitude=False, dyn_roll_only=False, dyn_sign=1.0, pitch_per_accel=0.0)
    BM.TS_ALL, _ = ply_frames(os.path.join(track_dir, 'raw_ply'))
    BM.MAP = BM.load_map(track_dir)
    BM.MARKS = MarkingMap(marks_npz)
    BM.BEV = _bev.ImageBEV(track_dir, 1.5, 0.0, 0.0, yaw_mount)
    BM.BEV.photonorm = False
    _bev.GROUND_MODE = 'profile'
    _bev.HYBRID_RATIO = 0.6
    BM.ROADBEV = None


def set_window(max_shift=None, max_yaw=None, yaw_step=None):
    if max_shift is not None:
        BM.A.max_shift = max_shift
    if max_yaw is not None:
        BM.A.max_yaw = max_yaw
    if yaw_step is not None:
        BM.A.yaw_step = yaw_step


def match(i, pose):
    return BM.match_frame(i, tuple(pose))


# ---- causal accumulation (online): past frames only, warped by the filter's own past poses -------------------------
_CACHE = {}


def _render_cached(j, fn):
    """Camera lane response of frame j (image only, pose independent): rendered once, kept for later frames."""
    if j not in _CACHE:
        _CACHE[j] = _RENDER(j, fn)
        if len(_CACHE) > 300:
            for k in sorted(_CACHE)[:50]:
                del _CACHE[k]
    return _CACHE[j]


def _causal_neighbours(i):
    """Frames accumulated with frame i: frames j <= i within accum_dist metres of travel behind (one per accum_ds);
    when nearly stationary, the last frames of a time window."""
    s = BM.PATH_S
    lo = int(np.searchsorted(s[:i + 1], s[i] - BM.A.accum_dist))
    cand = np.arange(lo, i + 1)
    key = np.floor((s[cand] - s[i]) / BM.A.accum_ds).astype(int)
    _, first = np.unique(key, return_index=True)
    sel = set(cand[first].tolist()) | {i}
    if len(sel) < 5:
        sel |= {j for j in range(i - BM.A.accum, i + 1, BM.A.accum_step) if j >= 0}
    return sorted(sel)


def setup_accum(n, accum_dist=15.0, accum_ds=0.7, accum=12, accum_step=3):
    global _RENDER
    BM.A.accum, BM.A.accum_dist, BM.A.accum_ds, BM.A.accum_step = accum, accum_dist, accum_ds, accum_step
    BM.PRIOR_REL = np.zeros((n, 3))
    BM.PATH_S = np.zeros(n)
    BM.neighbour_frames = _causal_neighbours
    _RENDER = BM.BEV.render_resp
    BM.BEV.render_resp = _render_cached


def set_pose(j, pose):
    """Store the (final) pose of frame j, or the current prior of the frame being matched."""
    BM.PRIOR_REL[j] = pose
    if j > 0:
        BM.PATH_S[j] = BM.PATH_S[j - 1] + float(np.hypot(pose[0] - BM.PRIOR_REL[j - 1][0], pose[1] - BM.PRIOR_REL[j - 1][1]))
