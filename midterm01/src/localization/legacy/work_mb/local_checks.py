"""Local (no leaderboard) checks for T2: (A) camera-fix vs GNSS forward offset vs speed (timing); (B) start yaw from
IMU-propagated GNSS course vs the lane-fix yaw."""
import sys, numpy as np, pandas as pd
sys.path.insert(0, 'tools')
from scipy.ndimage import gaussian_filter1d as g
G = pd.read_csv('work/_t2_gnssonly.csv')                       # GNSS + IMU only fusion (no map fixes)
z = dict(np.load('work/c16p20g_bm_t2m.npz')); i = z['idx'].astype(int)
dt = 1 / 30.0
x, y, yw = G.x.values, G.y.values, G.yaw.values
v = g(np.hypot(np.gradient(x), np.gradient(y)) / dt, 5)
c, s = np.cos(yw[i]), np.sin(yw[i]); dx, dy = z['mx'] - x[i], z['my'] - y[i]
fw = c * dx + s * dy
m = (i > 600) & (z['sharp_f'] > 0.03)
A = np.c_[np.ones(m.sum()), v[i][m]]
co = np.linalg.lstsq(A, fw[m], rcond=None)[0]
print(f'(A) lane fix - GNSS-only forward = {co[0]:+.3f} + {co[1]:+.4f} * speed  (n {m.sum()}, corr {np.corrcoef(fw[m], v[i][m])[0,1]:+.2f}) -> timing {-co[1]*1000:+.0f} ms (fix behind GNSS if +)')
for lo, hi in ((0, 8), (8, 12), (12, 15), (15, 25)):
    k = m & (v[i] >= lo) & (v[i] < hi)
    if k.sum() > 5: print(f'     speed {lo:2d}-{hi:2d} m/s: n {k.sum():3d} median fwd {np.median(fw[k]):+.3f}')
# (B) start yaw: course of the GNSS-only track while moving straight vs lane-fix yaw at the start
course = np.unwrap(np.arctan2(np.gradient(g(y, 15)), np.gradient(g(x, 15))))
imu = np.unwrap(yw)                                                 # fused yaw follows the IMU shape
seg = (np.arange(len(x)) > 400) & (np.arange(len(x)) < 700) & (v > 3)
off = np.degrees(np.median(course[seg] - imu[seg]))
print(f'(B) GNSS course - fused(IMU) yaw over 400-700: {off:+.2f} deg')
st = i < 250
print(f'    start lane-fix yaw - fused yaw: {np.degrees(np.median(np.unwrap(z["myaw"][st]) - imu[i[st]])):+.2f} deg; '
      f'implied start yaw from course (IMU-propagated) - fused start yaw: {off:+.2f} deg')
