"""Physical consistency (no leaderboard): along-track acceleration of a trajectory vs the IMU's gravity-compensated
forward acceleration (both smoothed 0.5 s). Also velocity: trajectory speed vs IMU-integrated speed changes."""
import sys, numpy as np, pandas as pd
from scipy.spatial.transform import Rotation
from scipy.ndimage import gaussian_filter1d as g
T = sys.argv[1]
im = pd.read_csv(f'/root/data/{T}/sensors/imu.csv')
idx = pd.read_csv(f'/root/data/{T}/3cameras_ply/index.csv')
ft = idx.f_ts.values.astype(np.float64)
it = im.stamp.values.astype(np.float64)
R = Rotation.from_quat(im[['qx', 'qy', 'qz', 'qw']].values)
aw = R.apply(im[['ax', 'ay', 'az']].values)                     # specific force in the world frame
grav = np.median(aw[:, 2])
aw[:, 2] -= grav
dt_i = np.median(np.diff(it)) * 1e-9
aws = np.stack([g(aw[:, k], 0.5 / dt_i) for k in range(2)], 1)
print(f'{T}: IMU rate {1/dt_i:.0f} Hz, gravity {grav:.3f}')
for f in sys.argv[2:]:
    D = pd.read_csv(f); n = len(D)
    t = (ft[:n] - ft[0]) * 1e-9
    x, y, yw = D.x.values, D.y.values, D.yaw.values
    dtf = np.median(np.diff(t))
    vx, vy = np.gradient(g(x, 0.1 / dtf), t), np.gradient(g(y, 0.1 / dtf), t)
    ax, ay = g(np.gradient(vx, t), 0.5 / dtf), g(np.gradient(vy, t), 0.5 / dtf)
    a_traj = np.cos(yw) * ax + np.sin(yw) * ay
    ai = np.stack([np.interp(ft[:n], it, aws[:, k]) for k in range(2)], 1)
    a_imu = np.cos(yw) * ai[:, 0] + np.sin(yw) * ai[:, 1]
    m = slice(30, n - 30)
    bias = np.median(a_traj[m] - a_imu[m])
    e = a_traj[m] - a_imu[m] - bias
    print(f'{f.split("/")[-1]:36s} along-track accel vs IMU: rms {np.sqrt(np.mean(e**2)):.3f} m/s2 (bias {bias:+.3f}), p95 {np.percentile(np.abs(e), 95):.3f}')
