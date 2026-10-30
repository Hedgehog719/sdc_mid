# deck figure: how the marking map finds the ground (Track02 start, queued cars in the map)
import sys
import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, 'tools')
from bev import local_ground
from bev_match import load_map
plt.rcParams.update({'font.size': 13, 'figure.facecolor': 'white'})
NAVY, SLATE, RUST = '#24364f', '#8796ab', '#b0643c'
r = np.loadtxt('submissions/final/result_2.csv', delimiter=',', skiprows=1)
x0, y0, yaw = r[0, 1:4]
M = load_map('/root/data/Track2')
idx = M.query_idx(x0, y0, 22.0)
P, I = M.pts[idx].astype(np.float64), np.asarray(M.inten)[idx].astype(np.float32)
c, s = np.cos(yaw), np.sin(yaw)
f = c * (P[:, 0] - x0) + s * (P[:, 1] - y0)
l = -s * (P[:, 0] - x0) + c * (P[:, 1] - y0)
R, half = 0.1, 15.0
n = int(2 * half / R)
ci = np.floor((half - l) / R).astype(int)      # rows: left at the top
cj = np.floor((f + half) / R).astype(int)      # cols: forward to the right
ok = (ci >= 0) & (ci < n) & (cj >= 0) & (cj < n)
ci, cj, z, it = ci[ok], cj[ok], P[ok, 2], I[ok]
k = ci * n + cj
zmin = np.full(n * n, np.inf); np.minimum.at(zmin, k, z); zmin = zmin.reshape(n, n)
valid = np.isfinite(zmin)
g = local_ground(np.where(valid, zmin, 0).astype(np.float32), valid, open_m=1.5, smooth_m=0.5)
zr = np.median(g)
hrel = z - g[ci, cj]
band = (hrel > -0.15) & (hrel < 0.20)
imax_all = np.zeros(n * n); np.maximum.at(imax_all, k, it); imax_all = imax_all.reshape(n, n)
imax_g = np.zeros(n * n); np.maximum.at(imax_g, k[band], it[band]); imax_g = imax_g.reshape(n, n)
top = np.full(n * n, np.nan); np.fmax.at(top, k, hrel); top = top.reshape(n, n)
ext = [-half, half, -half, half]
fig, ax = plt.subplots(2, 3, figsize=(18, 11.5))
im = ax[0, 0].imshow(np.where(valid, zmin - zr, np.nan), cmap='viridis', vmin=-0.3, vmax=1.5, extent=ext)
ax[0, 0].set_title('1. lowest point per 0.1 m cell (m)\ncar bodies still show up'); plt.colorbar(im, ax=ax[0, 0], fraction=0.046)
im = ax[0, 1].imshow(g - zr, cmap='viridis', vmin=-0.3, vmax=1.5, extent=ext)
ax[0, 1].set_title('2. after grey opening 1.5 m + smoothing\n= estimated ground surface'); plt.colorbar(im, ax=ax[0, 1], fraction=0.046)
im = ax[0, 2].imshow(top, cmap='magma', vmin=-0.2, vmax=2.5, extent=ext)
ax[0, 2].set_title('3. highest point above that ground (m)\ncars, roofs, walls'); plt.colorbar(im, ax=ax[0, 2], fraction=0.046)
ax[1, 0].imshow(imax_all, cmap='gray', vmin=0, vmax=np.percentile(imax_all[imax_all > 0], 99), extent=ext)
ax[1, 0].set_title('4. BEFORE: reflectivity of all points\n(cars and roofs mixed in)')
ax[1, 1].imshow(imax_g, cmap='gray', vmin=0, vmax=np.percentile(imax_g[imax_g > 0], 99), extent=ext)
ax[1, 1].set_title('5. AFTER: only the ground band -0.15..+0.20 m\n(road markings stand out)')
sl = np.abs(f - 6) < 1.5
ax[1, 2].scatter(l[sl], P[sl, 2] - zr, s=0.5, color=SLATE, label='map points', rasterized=True)
j = int((6 + half) / R)
lv = half - (np.arange(n) + 0.5) * R
ax[1, 2].plot(lv, np.where(valid[:, j], zmin[:, j] - zr, np.nan), '.', color=NAVY, ms=3, label='lowest point per cell')
ax[1, 2].plot(lv, g[:, j] - zr, color=RUST, lw=2.5, label='estimated ground')
ax[1, 2].fill_between(lv, g[:, j] - zr - 0.15, g[:, j] - zr + 0.20, color=RUST, alpha=0.25, label='kept band')
ax[1, 2].set_xlim(-12, 12); ax[1, 2].set_ylim(-0.6, 3); ax[1, 2].invert_xaxis()
ax[1, 2].set_xlabel('lateral (m, left +)'); ax[1, 2].set_ylabel('height (m)')
ax[1, 2].set_title('6. cross-section 6 m ahead'); ax[1, 2].legend(loc='upper right', fontsize=10, markerscale=5)
for a in ax.flat[:5]:
    a.set_xlabel('forward (m)'); a.set_ylabel('lateral (m)')
    a.plot(0, 0, marker='>', color='red', ms=12)
plt.suptitle('Track02 start: finding the ground in the LiDAR map (no trajectory used)', fontsize=16)
plt.tight_layout(); plt.savefig('work/deck/f_ground.png', dpi=100); plt.close()
print('ok')
