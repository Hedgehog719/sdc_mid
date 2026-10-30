# deck figure: camera rig on the car with viewing frusta, seen from behind-left and above (front-camera FLU frame)
import os, sys
import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
sys.path.insert(0, 'tools')
from camera_model import Cameras, R_OPT_FLU
plt.rcParams.update({'font.size': 13, 'figure.facecolor': 'white'})
C = Cameras('/root/data/Track3/Track3_images/camera_info')
H = 1.52                                    # front camera above the road (calibration)
col = {'fl': '#3a7bd5', 'f': '#24364f', 'fr': '#b0643c'}
name = {'fl': 'front-left', 'f': 'front', 'fr': 'front-right'}
fig = plt.figure(figsize=(14, 9)); ax = fig.add_subplot(111, projection='3d')
# road grid and the BEV area used for lane matching (-5..30 m ahead, +-15 m)
for gx in np.arange(-5, 8, 1):
    ax.plot([gx, gx], [-6, 6], [-H, -H], color='#e3e6ea', lw=0.6)
for gy in np.arange(-6, 7, 1):
    ax.plot([-5, 7], [gy, gy], [-H, -H], color='#e3e6ea', lw=0.6)
# car body (illustrative size: 4.6 x 1.8 x 1.45 m; cameras on the roof front)
x0, x1, w, zb, zt = -3.3, 1.3, 0.9, -H + 0.15, -0.08
V = np.array([[x0, -w, zb], [x1, -w, zb], [x1, w, zb], [x0, w, zb], [x0, -w, zt], [x1, -w, zt], [x1, w, zt], [x0, w, zt]])
faces = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
ax.add_collection3d(Poly3DCollection([V[f] for f in faces], facecolor='#d9dde3', edgecolor='#555555', alpha=0.35, lw=0.8))
ax.plot([x1, x1 + 0.6], [0, 0], [zb + 0.4, zb + 0.4], color='#555555', lw=2)   # nose marker
for cam in ('fl', 'f', 'fr'):
    K, D, T, iw, ih = C.cams[cam]
    R, t = T[:3, :3], T[:3, 3]                 # front-optical -> camera-optical
    if cam in C.rot_fix:
        pass                                   # corrections are a few tenths of a degree: not visible at this scale
    c_opt = -R.T @ t                           # camera centre in the front optical frame
    c = R_OPT_FLU.T @ c_opt
    if cam in C.trans_fix:
        c = c + C.trans_fix[cam]
    fx = K[0, 0]
    ah, av = (iw / 2) / fx, (ih / 2) / fx      # fisheye (equidistant) half angles
    rng = 3.5
    corners = []
    for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        th = np.hypot(sx * ah, sy * av); ph = np.arctan2(sy * av, sx * ah)
        th = min(th, np.radians(80))
        d_opt = np.array([np.sin(th) * np.cos(ph), np.sin(th) * np.sin(ph), np.cos(th)])
        corners.append(c + rng * (R_OPT_FLU.T @ (R.T @ d_opt)))
    axis = c + rng * (R_OPT_FLU.T @ (R.T @ np.array([0, 0, 1.0])))
    tri = [[c, corners[i], corners[(i + 1) % 4]] for i in range(4)]
    ax.add_collection3d(Poly3DCollection(tri, facecolor=col[cam], edgecolor=col[cam], alpha=0.12, lw=0.6))
    ax.plot(*np.array([c, axis]).T, color=col[cam], lw=2.2)
    ax.scatter(*c, color=col[cam], s=60, depthshade=False)
    yaw = np.degrees(np.arctan2(axis[1] - c[1], axis[0] - c[0]))
    off = {'fl': [0, 0.6, 0.6], 'f': [0.4, 0, 0.9], 'fr': [0, -0.4, -0.9]}[cam]
    ax.text(*(axis + np.array(off)), f'{name[cam]}\nyaw {yaw:+.0f} deg, HFOV ~{2*np.degrees(ah):.0f} deg',
            color=col[cam], fontsize=12)
    print(cam, 'centre (fwd, left, up) m', np.round(c, 3), 'axis yaw deg', round(yaw, 1))
ax.set_xlim(-5, 7); ax.set_ylim(-6, 6); ax.set_zlim(-H, 2.5)
ax.set_box_aspect((12, 12, 4))
ax.set_xlabel('forward (m)'); ax.set_ylabel('left (m)'); ax.set_zlabel('up (m)')
ax.view_init(elev=20, azim=-145)
ax.set_title('Camera rig (provided extrinsics, front camera 1.52 m above the road); car body size illustrative', fontsize=13)
plt.tight_layout(); plt.savefig('work/deck/f_rig.png', dpi=110); plt.close()
