"""Rehearsal check: a GNSS-free trajectory vs the reference (best GNSS-aided pipeline output), per segment."""
import sys
import numpy as np, pandas as pd
A, R = pd.read_csv(sys.argv[1]), pd.read_csv(sys.argv[2])
n = min(len(A), len(R)); A, R = A.iloc[:n], R.iloc[:n]
yw = R.yaw.values
dx, dy = A.x.values - R.x.values, A.y.values - R.y.values
fw, lt = np.cos(yw) * dx + np.sin(yw) * dy, -np.sin(yw) * dx + np.cos(yw) * dy
dyaw = np.degrees((A.yaw.values - yw + np.pi) % (2 * np.pi) - np.pi)
e = np.hypot(fw, lt)
print(f'{sys.argv[1]} vs {sys.argv[2]}: position rms {np.sqrt(np.mean(e**2)):.3f} m (fwd {np.sqrt(np.mean(fw**2)):.3f}, lat {np.sqrt(np.mean(lt**2)):.3f}), max {e.max():.2f} m at {e.argmax()}; yaw rms {np.sqrt(np.mean(dyaw**2)):.3f} deg; score-like {np.sqrt(np.mean(e**2)) + 0.2 * np.sqrt(np.mean(dyaw**2)):.3f}')
step = int(sys.argv[3]) if len(sys.argv) > 3 else 300
print('   per segment fwd/lat mean | yaw mean: ' + ' '.join(f'{s0}:{fw[s0:s0+step].mean():+.2f}/{lt[s0:s0+step].mean():+.2f}|{dyaw[s0:s0+step].mean():+.1f}' for s0 in range(0, n, step)))
