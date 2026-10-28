# analysis: the --terrain_lat lateral residual grid (map road height beside the lane strip relative to the vehicle
# plane + along-track profile) at given frames of a trajectory, as tools/bev.py terrain_dz computes it
import sys
import numpy as np
import pandas as pd
sys.path.insert(0, 'tools')
from bev import X_MIN, X_MAX, Y_MAX  # noqa: E402
from marking_map import MarkingMap  # noqa: E402
MK = MarkingMap('work/runs/T3/mk_D.npz')
tr = pd.read_csv(sys.argv[1])
frames = [int(v) for v in sys.argv[2:]]


def F(u):
    u = np.asarray(u, float) / 10.0
    return np.stack([u, u * u, u ** 3], 1)


for k in frames:
    x, y, yaw = tr.x[k], tr.y[k], tr.yaw[k]
    c, s = np.cos(yaw), np.sin(yaw)
    world = lambda u, v: (x + c * u - s * v, y + s * u + c * v)  # noqa: E731
    ou, ov = (g.ravel() for g in np.meshgrid(np.linspace(-4, 4, 9), np.linspace(-2, 2, 5)))
    pz, ok = MK.height(*world(ou, ov))
    ok &= np.isfinite(pz)
    pa = np.linalg.lstsq(np.stack([ou[ok], ov[ok], np.ones(ok.sum())], 1), pz[ok], rcond=None)[0]
    su = np.arange(X_MIN, X_MAX + 0.1, 1.0)
    U, V = np.meshgrid(su, np.arange(-2.0, 2.1, 1.0))
    z, okz = MK.height(*world(U.ravel(), V.ravel()))
    z = np.where(okz & np.isfinite(z), z - pa[1] * V.ravel(), np.nan).reshape(U.shape)
    prof = np.array([np.nanmedian(col) if np.isfinite(col).any() else np.nan for col in z.T])
    okp = np.isfinite(prof)
    uu = su[okp]
    d = prof[okp] - (pa[0] * uu + pa[2])
    keep = np.abs(d) < 0.12 + 0.015 * np.abs(uu)
    for _ in range(5):
        coef = np.linalg.lstsq(F(uu[keep]), d[keep], rcond=None)[0]
        r = d - F(uu) @ coef
        keep = np.abs(r) < max(3 * 1.4826 * np.median(np.abs(r[keep])), 0.03)
    cu, cv = np.arange(X_MIN, X_MAX + 0.01, 1.0), np.arange(-Y_MAX, Y_MAX + 0.01, 0.5)
    CU, CV = np.meshgrid(cu, cv, indexing='ij')
    zc, okc = MK.height(*world(CU.ravel(), CV.ravel()))
    res = zc.reshape(CU.shape) - (pa[0] * CU + pa[1] * CV + pa[2]) - (F(cu) @ coef)[:, None]
    res = np.where(okc.reshape(CU.shape) & np.isfinite(res), res, np.nan)
    sel = (cu >= 5) & (cu <= 20)
    lat = np.nanmedian(res[sel], axis=0)
    print(f'frame {k}: crossfall {np.degrees(np.arctan(pa[1])):+.2f} deg | lateral residual (5-20 m ahead, v from -{Y_MAX:.0f} to +{Y_MAX:.0f} m step 0.5, + = left):')
    print('   ', ' '.join(f'{v:+.2f}' if np.isfinite(v) else '  nan' for v in lat))
