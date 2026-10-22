"""Single-frame lane matching (bev_match.match_frame without accumulation) as a library: for sequential use without
a prior trajectory (Track03 tracker). setup() loads the map / marking map / camera BEV once; match(i, pose) returns
(peak, sharp, fwd, lat, dyaw, sharp_f, sharp_l) = the correction of pose (vehicle frame) or None."""
import argparse
import os
import sys

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
