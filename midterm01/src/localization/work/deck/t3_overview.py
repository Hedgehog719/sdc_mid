# deck figure: Track03 route over the LiDAR map (top view) with the frames discussed in the talk
import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 14, 'figure.facecolor': 'white'})
NAVY, SLATE, RUST = '#24364f', '#8796ab', '#b0643c'
tr = np.loadtxt('submissions/final/result_3.csv', delimiter=',', skiprows=1)
X, Y = tr[:, 1], tr[:, 2]
pad = 25
m = np.load('/root/data/Track3/map.npy', mmap_mode='r')
pts = []
for i in range(0, len(m), 5_000_000):
    c = np.asarray(m[i:i + 5_000_000])
    k = (c[:, 0] > X.min() - pad) & (c[:, 0] < X.max() + pad) & (c[:, 1] > Y.min() - pad) & (c[:, 1] < Y.max() + pad)
    pts.append(c[k][::20])
P = np.concatenate(pts)
fig, ax = plt.subplots(figsize=(16, 10))
ax.scatter(P[:, 0], P[:, 1], s=0.05, color='#c9d1dc', rasterized=True)
seg = [(0, 600, NAVY, 'start, 5 heading hypotheses (0-599)'), (600, 2280, SLATE, ''), (2280, 2490, RUST, 'lateral disagreement'),
       (2490, 2790, SLATE, ''), (2790, 2970, RUST, ''), (2970, len(X), SLATE, '')]
for a, b, col, lab in seg:
    ax.plot(X[a:b], Y[a:b], color=col, lw=3.5 if col != SLATE else 2.5, label=lab or None, solid_capstyle='round')
marks = [(0, 'start (frame 0)'), (720, '720 kerb-side gutter'), (879, '879 braking dive'),
         (1600, '1200-2000 red light, 27 s'), (2458, '2458 heading passes 180 deg'), (2600, '2600 box junction'),
         (len(X) - 1, 'end (3141)')]
offs = [(30, 20), (30, -30), (-60, 55), (60, 25), (-230, 25), (30, -35), (30, 20)]
for (k, txt), (dx, dy) in zip(marks, offs):
    ax.plot(X[k], Y[k], 'o', color='black', ms=7)
    ax.annotate(txt, (X[k], Y[k]), xytext=(dx, dy), textcoords='offset points', fontsize=13,
                arrowprops=dict(arrowstyle='-', lw=0.8, color='#555555'))
for k in range(0, len(X), 300):   # direction ticks
    if k + 5 < len(X):
        ax.annotate('', xy=(X[k + 5], Y[k + 5]), xytext=(X[k], Y[k]), arrowprops=dict(arrowstyle='->', color=NAVY, lw=1.5))
ax.set_aspect('equal'); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
L = np.hypot(np.diff(X), np.diff(Y)).sum()
ax.set_title(f'Track03 route (final output) over the LiDAR map, {len(X)} frames, {L:.0f} m driven')
ax.legend(loc='best', frameon=False)
plt.tight_layout(); plt.savefig('work/deck/f_t3_overview.png', dpi=100); plt.close()
print('ok', round(L))
