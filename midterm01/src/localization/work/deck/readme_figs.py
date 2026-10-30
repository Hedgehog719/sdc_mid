# README figures: (1) correlation search on a real frame, (2) flat vs terrain projection schematic
import sys
import cv2, numpy as np, pandas as pd, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, 'tools')
from bev import ImageBEV, lane_response, RES
from marking_map import MarkingMap
plt.rcParams.update({'font.size': 13, 'figure.facecolor': 'white'})
NAVY, SLATE, RUST = '#24364f', '#8796ab', '#b0643c'

# 1. correlation search, Track03 frame 2600, at a pose shifted by a known offset
tr = pd.read_csv('submissions/final/result_3.csv'); dd = pd.read_csv('work/runs/FINAL_T3/debug.csv')
DY = np.zeros((len(tr), 2))
for j, c in enumerate(('att_p', 'att_r')):
    v = dd[c].values[:len(tr)]; DY[:len(v), j] = np.nan_to_num(v.astype(float))
MK = MarkingMap('work/runs/T3/mk_D.npz')
B = ImageBEV('/root/data/Track3', 1.5, 0, 0, -1.5); B.dyn = DY; B.terrain = MK.height; B.TERRAIN_LAT = True
k = 2600
x, y, yaw = tr.x[k], tr.y[k], tr.yaw[k]
B.pose = (x, y, yaw)
resp, valid = B.render_resp(k, lane_response)
dfw, dlt = 0.8, -0.5                                  # pretend the prediction is 0.8 m behind and 0.5 m off
c_, s_ = np.cos(yaw), np.sin(yaw)
xp, yp = x - c_ * dfw + s_ * dlt, y - s_ * dfw - c_ * dlt
mp = MK.bev(np.array([xp, yp]), yaw)[0].astype(np.float32)
P = int(2.5 / RES)
pad = cv2.copyMakeBorder(mp, P, P, P, P, cv2.BORDER_CONSTANT, value=0)
corr = cv2.matchTemplate(pad, resp.astype(np.float32), cv2.TM_CCOEFF_NORMED)
r0, c0 = np.unravel_index(np.argmax(corr), corr.shape)
fig, ax = plt.subplots(1, 3, figsize=(18, 6.2))
ax[0].imshow(np.rot90(np.clip(resp / 2, 0, 1), -1), cmap='gray'); ax[0].set_title('camera markings in BEV (white top-hat)')
ax[1].imshow(np.rot90(mp, -1), cmap='Reds'); ax[1].set_title('LiDAR marking map at the predicted pose\n(predicted 0.8 m behind, 0.5 m off)')
ext = [-2.5, 2.5, -2.5, 2.5]
im = ax[2].imshow(np.rot90(corr, -1), cmap='viridis', extent=ext, origin='upper')
ax[2].set_xlabel('forward shift (m)'); ax[2].set_ylabel('lateral shift (m)')
ax[2].set_title(f'correlation over all shifts, peak {corr.max():.2f}')
plt.colorbar(im, ax=ax[2], fraction=0.046)
for a in ax[:2]:
    a.set_xticks([]); a.set_yticks([]); a.set_xlabel('car on the left, driving right')
plt.tight_layout(); plt.savefig('work/deck/f_corr.png', dpi=100); plt.close()
print('peak at row/col', r0, c0, 'shift m', (r0 - P) * RES, (c0 - P) * RES)

# 2. why a flat ground plane misplaces far markings
h, d, dz = 1.52, 18.0, 0.15
fig, ax = plt.subplots(figsize=(14, 4.8))
xs = np.linspace(0, 24, 100)
road = np.where(xs < 8, 0, (xs - 8) * dz / (d - 8))
ax.fill_between(xs, -0.4, road, color='#e3e6ea')
ax.plot(xs, road, color=NAVY, lw=2, label='real road (rises 0.15 m)')
ax.plot([0, 24], [0, 0], color=SLATE, lw=1.5, ls='--', label='flat assumption')
ax.plot(0, h, 'o', color='black', ms=9); ax.text(0.3, h + 0.08, 'camera, 1.52 m', fontsize=13)
dflat = d * h / (h - dz)
ax.plot([0, dflat], [h, 0], color=RUST, lw=1.5)
ax.plot(d, dz, 'o', color=NAVY, ms=9); ax.text(d - 2.2, dz + 0.12, 'marking at 18.0 m', color=NAVY)
ax.plot(dflat, 0, 'x', color=RUST, ms=12, mew=3); ax.text(dflat - 0.8, -0.3, f'flat IPM puts it at {dflat:.1f} m', color=RUST)
ax.set_xlim(-0.5, 24); ax.set_ylim(-0.4, 1.9); ax.set_xlabel('distance ahead (m)'); ax.set_ylabel('height (m)')
ax.set_title('Same pixel, two ground models: d_flat = d * h / (h - dz)  ->  terrain-aware projection uses the map road height')
ax.legend(loc='upper right', frameon=False)
for sp in ('top', 'right'):
    ax.spines[sp].set_visible(False)
plt.tight_layout(); plt.savefig('work/deck/f_ipm.png', dpi=100); plt.close()
print('ok')
