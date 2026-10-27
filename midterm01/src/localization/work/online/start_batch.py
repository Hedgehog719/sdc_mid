# T3 start reference by batch least squares (validation only, never written to a submission): the start pose (x0, y0,
# yaw0) and the VO scale that best explain all recorded lane matches of frames 0..N, the path in between from the VO
# pair displacements and the gyro heading; then each candidate's start error vs this reference, ranked against public
# usage: start_batch.py <rec.npz> <N> [--lat]
import glob
import json
import sys
import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from scipy.stats import spearmanr
sys.path.insert(0, 'tools')
from common import ply_frames  # noqa: E402
REC = np.load(sys.argv[1], allow_pickle=True)['rec'].item()
N = int(sys.argv[2])
USE_LAT = '--lat' in sys.argv
FIX_S = float(sys.argv[sys.argv.index('--scale') + 1]) if '--scale' in sys.argv else None
TS = np.asarray(ply_frames('/root/data/Track3/raw_ply')[0], np.int64)
im = pd.read_csv('/root/data/Track3/sensors/imu.csv')
q = Rotation.from_quat(im[['qx', 'qy', 'qz', 'qw']].values)
wz = q.apply(im[['wx', 'wy', 'wz']].values)[:, 2] - np.radians(-0.23)
ti = im.t.values.astype(np.int64)
yg = np.concatenate([[0], np.cumsum(0.5 * (wz[1:] + wz[:-1]) * np.diff(ti) * 1e-9)])
psi = np.interp(TS[:N + 1], ti, yg)
psi -= psi[0]
z = np.load('work/runs/T3/vo_fill.npz')
V = {int(j): (f, l, r) for j, f, l, r in zip(z['j'], z['fwd'], z['lat'], z['resp'])}
dbg = pd.read_csv('work/runs/T3_vofill/debug.csv').set_index('k')
step_f, step_l = np.zeros(N), np.zeros(N)
for j in range(N):
    if j in V and V[j][2] > 0.5:
        step_f[j], step_l[j] = V[j][0], (V[j][1] if USE_LAT else 0.0)
    else:   # no VO pair: filter speed x interval (start, before the VO-initialised speed)
        step_f[j] = dbg.v.get(j + 1, 0.0) * (TS[j + 1] - TS[j]) * 1e-9
meas = []
for k, (ax, ay, ayaw, sf, sl, pk) in REC['lane'].items():
    if k <= N and pk > 0.2:
        meas.append((k, ax, ay, ayaw, sf, sl, pk))
meas = np.array(meas)
print(f'{len(meas)} lane matches with peak > 0.2 in frames 0-{N}; VO pairs missing {sum(1 for j in range(N) if not (j in V and V[j][2] > 0.5))}')


def path(th):
    x0, y0, h0, s = th if FIX_S is None else (*th[:3], FIX_S)
    h = h0 + psi
    hm = 0.5 * (h[:-1] + h[1:])
    dx = s * (np.cos(hm) * step_f - np.sin(hm) * step_l)
    dy = s * (np.sin(hm) * step_f + np.cos(hm) * step_l)
    return np.r_[x0, x0 + np.cumsum(dx)], np.r_[y0, y0 + np.cumsum(dy)], h


def resid(th):
    X, Y, H = path(th)
    k = meas[:, 0].astype(int)
    ex, ey = meas[:, 1] - X[k], meas[:, 2] - Y[k]
    c, s_ = np.cos(H[k]), np.sin(H[k])
    ef, el = c * ex + s_ * ey, -s_ * ex + c * ey
    eh = (meas[:, 3] - np.radians(0.3) - H[k] + np.pi) % (2 * np.pi) - np.pi
    sf, sl, pk = meas[:, 4], meas[:, 5], meas[:, 6]
    wf = np.where(sf > 0.02, 1.0 / np.clip(0.05 / np.sqrt(np.maximum(sf, 1e-6)), 0.08, 1.0), 0.0)
    wl = np.where(sl > 0.1, 1.0 / np.clip(0.005 / np.maximum(sl, 1e-6), 0.1, 1.0), 0.0)
    wh = np.where(pk > 0.25, 1.0 / np.radians(0.7), 0.0)
    return np.r_[ef * wf, el * wl, eh * wh]


o0 = pd.read_csv('submissions/online/t3_online_vofill.csv')
th0 = np.array([o0.x[0], o0.y[0], np.unwrap(o0.yaw.values)[0], 1.0 / max(dbg.kvo.get(30, 1.0), 0.5)])
best = None
for dh in (-2, 0, 2):
    for dfw in (-1.5, 0, 1.5):
        t = th0.copy()
        t[2] += np.radians(dh)
        t[0] += dfw * np.cos(t[2])
        t[1] += dfw * np.sin(t[2])
        r = least_squares(resid, t if FIX_S is None else t[:3], loss='soft_l1', f_scale=1.0)
        if best is None or r.cost < best.cost:
            best = r
th = best.x
X, Y, H = path(th)
c0, s0 = np.cos(H[0]), np.sin(H[0])
dx, dy = o0.x[0] - X[0], o0.y[0] - Y[0]
print(f'fit: VO scale {th[3] if FIX_S is None else FIX_S:.4f}; vofill frame 0 minus reference: fwd {c0 * dx + s0 * dy:+.2f} lat {-s0 * dx + c0 * dy:+.2f} m, '
      f'yaw {np.degrees(np.unwrap(o0.yaw.values)[0] - H[0]):+.2f} deg; cost {best.cost:.1f}')


def err(o, n):
    k = np.arange(n)
    h = H[k]
    ex, ey = o.x.values[:n] - X[k], o.y.values[:n] - Y[k]
    eh = np.degrees((o.yaw.values[:n] - h + np.pi) % (2 * np.pi) - np.pi)
    return np.sqrt((ex ** 2 + ey ** 2).mean()) + 0.2 * np.sqrt((eh ** 2).mean())


rows = []
for js in glob.glob('submissions/online/t3_*.json'):
    try:
        sc = float(json.load(open(js)).get('score'))
        o = pd.read_csv(js.replace('.json', '.csv'))
    except (TypeError, ValueError, FileNotFoundError):
        continue
    if len(o) > N:
        rows.append((js.split('/')[-1][:-5], sc, err(o, 30), err(o, 60), err(o, min(N, 150))))
if '--cand' in sys.argv:
    for cp in sys.argv[sys.argv.index('--cand') + 1:]:
        o = pd.read_csv(cp)
        print(f'candidate {cp}: e30 {err(o, 30):.3f} e60 {err(o, 60):.3f} e150 {err(o, min(N, 150)):.3f}')
        if '--prof' in sys.argv:
            for k in (0, 3, 6, 10, 15, 20, 30, 45, 60, 90):
                h = H[k]
                ex, ey = o.x[k] - X[k], o.y[k] - Y[k]
                print(f'    frame {k:3d}: fwd {np.cos(h) * ex + np.sin(h) * ey:+.2f} lat {-np.sin(h) * ex + np.cos(h) * ey:+.2f} '
                      f'yaw {np.degrees((o.yaw[k] - h + np.pi) % (2 * np.pi) - np.pi):+.2f}')
r = pd.DataFrame(rows, columns=['file', 'public', 'e30', 'e60', 'e150']).sort_values('public')
print(r.to_string(index=False, float_format=lambda v: f'{v:.3f}'))
for c_ in ('e30', 'e60', 'e150'):
    print(c_, 'spearman vs public %.2f' % spearmanr(r.public, r[c_]).correlation)
