"""Project points of the front-camera FLU cloud frame into the three fisheye cameras."""
import os

import cv2
import numpy as np
import yaml

# FLU (x fwd, y left, z up) -> optical (x right, y down, z fwd)
R_OPT_FLU = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=np.float64)
CAM_FILES = {'f': 'gige_100_f_hdr', 'fl': 'gige_100_fl_hdr', 'fr': 'gige_100_fr_hdr'}
EXTR_KEYS = {'fl': 'T_front_front_left', 'fr': 'T_front_front_right'}


class Cameras:
    def __init__(self, info_dir, rot_fix_file=None):
        extr = yaml.safe_load(open(os.path.join(info_dir, 'lucid_camera_extrinsics.yaml')))['transforms']
        self.cams = {}
        for cam, stem in CAM_FILES.items():
            info = yaml.safe_load(open(os.path.join(info_dir, f'{stem}_camera_info.yaml')))
            K = np.array(info['camera_matrix']['data'], np.float64).reshape(3, 3)
            D = np.array(info['distortion_coefficients']['data'], np.float64)
            T_cam_front = np.eye(4)
            if cam in EXTR_KEYS:
                T_cam_front = np.linalg.inv(np.array(extr[EXTR_KEYS[cam]]['matrix'], np.float64))
            self.cams[cam] = (K, D, T_cam_front, info['image_width'], info['image_height'])
        # per-camera rotation corrections from calib_side.py (work/cam_rot_fix.json), if present
        import json
        f = rot_fix_file or os.environ.get('SDC_ROT_FIX') or \
            os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'work', 'cam_rot_fix.json')
        if os.path.exists(f):
            d = json.load(open(f))
            self.rot_fix = {k: tuple(v) for k, v in d.items() if not k.endswith('_t')}
            # optional camera position corrections '<cam>_t': [dx, dy, dz] m in the front FLU frame
            self.trans_fix = {k[:-2]: np.array(v, float) for k, v in d.items() if k.endswith('_t')}

    yaw_fix = {'f': 0.0, 'fl': 0.5, 'fr': -0.5}  # deg, from calib_cam_yaw.py on Track2 poles
    rot_fix = {}  # optional per-camera (yaw, pitch, roll) deg corrections (calib_side.py); overrides yaw_fix
    trans_fix = {}  # optional per-camera position corrections (m, front FLU): true camera = nominal + t

    def project(self, cam, pts_flu):
        """Return pixel coords (N,2) and a validity mask for points given in the front FLU frame."""
        K, D, T, w, h = self.cams[cam]
        if cam in self.trans_fix:
            pts_flu = pts_flu - self.trans_fix[cam]
        if cam in self.rot_fix:
            from scipy.spatial.transform import Rotation
            yw, pt, rl = self.rot_fix[cam]
            pts_flu = pts_flu @ Rotation.from_euler('ZYX', [yw, pt, rl], degrees=True).as_matrix()
        elif self.yaw_fix.get(cam, 0.0):
            a = np.radians(self.yaw_fix[cam])
            Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1.0]])
            pts_flu = pts_flu @ Rz
        p = pts_flu @ R_OPT_FLU.T
        p = p @ T[:3, :3].T + T[:3, 3]
        valid = p[:, 2] > 0.3
        uv = np.full((len(p), 2), -1.0)
        if valid.any():
            q = p[valid]
            # keep within ~95 deg from axis to avoid fisheye wrap-around
            valid_idx = np.flatnonzero(valid)
            ang = np.arctan2(np.linalg.norm(q[:, :2], axis=1), q[:, 2])
            ok = ang < np.radians(95)
            proj, _ = cv2.fisheye.projectPoints(q[ok].reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D)
            uv[valid_idx[ok]] = proj.reshape(-1, 2)
            valid[valid_idx[~ok]] = False
        valid &= (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
        return uv, valid
