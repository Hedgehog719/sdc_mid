# map problems for the report deck: T1 start height drift, parked-car roofs in the map vs the ground band used
import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 15, 'axes.spines.top': False, 'axes.spines.right': False, 'figure.facecolor': 'white'})
O = 'work/deck/'
NAVY, SLATE, LIGHT, RUST = '#24364f', '#8796ab', '#c9d1dc', '#b0643c'


def crop(track, xmin, xmax, ymin, ymax):
    m = np.load(f'/root/data/{track}/map.npy', mmap_mode='r')
    out = []
    for i in range(0, len(m), 5_000_000):
        c = np.asarray(m[i:i + 5_000_000])
        k = (c[:, 0] > xmin) & (c[:, 0] < xmax) & (c[:, 1] > ymin) & (c[:, 1] < ymax)
        out.append(c[k])
    return np.concatenate(out)


def path_frame(P, xy):
    """map points -> (distance along the driven path, lateral offset, z) for points within the path's reach"""
    seg = np.diff(xy, axis=0)
    s = np.concatenate([[0], np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))])
    d2 = ((P[:, None, :2] - xy[None, ::5]) ** 2).sum(-1)
    j = d2.argmin(1) * 5
    j = np.clip(j, 0, len(xy) - 2)
    t = seg[j] / np.maximum(np.hypot(seg[j, 0], seg[j, 1]), 1e-6)[:, None]
    r = P[:, :2] - xy[j]
    return s[j] + (r * t).sum(1), -r[:, 0] * t[:, 1] + r[:, 1] * t[:, 0], P[:, 2]


# 1. Track01 start: road height along the path (mapping car stood here -> SLAM height drift, doubled ground)
r = np.loadtxt('submissions/final/result_1.csv', delimiter=',', skiprows=1)
xy = r[:420, 1:3]
P = crop('Track1', xy[:, 0].min() - 8, xy[:, 0].max() + 8, xy[:, 1].min() - 8, xy[:, 1].max() + 8)
P = P[::3]
s, l, z = path_frame(P, xy)
k = np.abs(l) < 2.5
fig, ax = plt.subplots(figsize=(14, 5.2))
ax.scatter(s[k], z[k], s=0.3, color=SLATE, rasterized=True)
ax.set_xlabel('distance along the driven path from the start (m)'); ax.set_ylabel('map height (m)')
ax.set_title('Track01 start: map points within 2.5 m of the path')
ax.annotate('roof layer ~1.7 m above the road, along the whole path', (50, 110.05), xytext=(30, 111.6), fontsize=13, color=RUST, arrowprops=dict(arrowstyle='->', color=RUST))
ax.annotate('road', (50, 108.2), xytext=(52, 107.2), fontsize=13, color=NAVY, arrowprops=dict(arrowstyle='->', color=NAVY))
ax.annotate('start: mapping car standing,\nsmeared / drifting heights', (5, 107.0), xytext=(-12, 106.7), fontsize=13, color=RUST)
zs = np.percentile(z[k & (s > 60)], 5)
ax.set_ylim(zs - 1.5, zs + 4.5)
plt.tight_layout(); plt.savefig(O + 'f_t1_mapdrift.png', dpi=110); plt.close()

# 2. Track02 start: a cross-section with the parked / queued cars in the map, and the ground band the marking map keeps
r = np.loadtxt('submissions/final/result_2.csv', delimiter=',', skiprows=1)
x0, y0, yaw0 = r[0, 1:4]
P = crop('Track2', x0 - 15, x0 + 15, y0 - 15, y0 + 15)
c, sn = np.cos(yaw0), np.sin(yaw0)
f = c * (P[:, 0] - x0) + sn * (P[:, 1] - y0)
lt = -sn * (P[:, 0] - x0) + c * (P[:, 1] - y0)
k = (np.abs(f - 6) < 1.5) & (np.abs(lt) < 12)
zg = np.percentile(P[k & (np.abs(lt) < 1.5), 2], 10)
zz = P[k, 2] - zg
band = (zz > -0.15) & (zz < 0.20)
fig, ax = plt.subplots(figsize=(14, 5.2))
ax.scatter(lt[k][~band], zz[~band], s=0.6, color=SLATE, label='other map points (cars, roofs, walls)', rasterized=True)
ax.scatter(lt[k][band], zz[band], s=0.6, color=RUST, label='ground band used for the marking map (-0.15..+0.20 m)', rasterized=True)
ax.set_xlabel('lateral offset from the car (m, left +)'); ax.set_ylabel('height above road (m)')
ax.set_title('Track02 start, cross-section 6 m ahead: cars and their roofs are in the map')
ax.set_ylim(-1, 4.5); ax.legend(frameon=False, loc='upper left', markerscale=12)
plt.tight_layout(); plt.savefig(O + 'f_roofs.png', dpi=110); plt.close()
