"""Structure of (pipeline - 0.17 file): position fwd/lat and yaw differences vs speed, acceleration, yaw rate,
curvature and position along the track (diagnosis only: which physical effect to look for in local data)."""
import sys, numpy as np, pandas as pd
A = pd.read_csv(sys.argv[1]); B = pd.read_csv(sys.argv[2]); dt = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0333
yw = B.yaw.values; dx, dy = A.x.values - B.x.values, A.y.values - B.y.values
fw, lt = np.cos(yw) * dx + np.sin(yw) * dy, -np.sin(yw) * dx + np.cos(yw) * dy
da = np.degrees((A.yaw.values - yw + np.pi) % (2 * np.pi) - np.pi)
x, y = B.x.values, B.y.values
v = np.hypot(np.gradient(x), np.gradient(y)) / dt
acc = np.gradient(v) / dt
yr = np.degrees(np.gradient(np.unwrap(yw))) / dt
from scipy.ndimage import gaussian_filter1d as g
v, acc, yr = g(v, 5), g(acc, 15), g(yr, 5)
n = len(B); s = slice(330, n)          # outside the start
print(f'n {n}, speed median {np.median(v):.1f} m/s, |acc| p90 {np.percentile(np.abs(acc), 90):.2f}, |yaw rate| p90 {np.percentile(np.abs(yr), 90):.2f} deg/s')
for nm, e in (('fwd', fw), ('lat', lt), ('yaw', da)):
    X = np.c_[np.ones(n), v, acc, yr, np.abs(yr)][s]
    c, *_ = np.linalg.lstsq(X, e[s], rcond=None)
    r = e[s] - X @ c
    print(f'{nm}: rms {np.sqrt(np.mean(e[s]**2)):.3f} | fit const {c[0]:+.3f} speed {c[1]:+.4f}/(m/s) acc {c[2]:+.3f}/(m/s2) yawrate {c[3]:+.4f}/(deg/s) |yawrate| {c[4]:+.4f} -> residual rms {np.sqrt(np.mean(r**2)):.3f}')
    corr = {k: np.corrcoef(e[s], q[s])[0, 1] for k, q in (('speed', v), ('acc', acc), ('yawrate', yr))}
    print('     correlations: ' + ' '.join(f'{k} {c_:+.2f}' for k, c_ in corr.items()))
# low-frequency share: how much of the difference is slow (> 300 frames) vs fast
for nm, e in (('fwd', fw), ('lat', lt), ('yaw', da)):
    slow = g(e, 100)
    print(f'{nm}: slow part (sigma 100 frames) rms {np.sqrt(np.mean(slow[s]**2)):.3f}, fast part rms {np.sqrt(np.mean((e - slow)[s]**2)):.3f}')
for a in range(0, n, 300):
    b = min(a + 300, n)
    print(f'  {a:4d}-{b:4d}: fwd {fw[a:b].mean():+.3f} (rms {np.sqrt(np.mean(fw[a:b]**2)):.3f}) lat {lt[a:b].mean():+.3f} yaw {da[a:b].mean():+.3f} (rms {np.sqrt(np.mean(da[a:b]**2)):.3f}) | v {v[a:b].mean():4.1f} yr {yr[a:b].mean():+.2f}')
