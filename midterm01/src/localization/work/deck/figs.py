# figures for the report deck: plain, white background, black text
import pandas as pd, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
plt.rcParams.update({'font.size': 15, 'axes.spines.top': False, 'axes.spines.right': False, 'figure.facecolor': 'white'})
O = 'work/deck/'

# 1. public score history
fig, axs = plt.subplots(1, 3, figsize=(18, 5.2))
for ax, (c, lab, lim) in zip(axs, [('sdc-2026-midterm-01-track-01', 'Track01', (0.3, 1.4)), ('sdc-2026-localization-track-02', 'Track02', (0.1, 0.6)), ('sdc-2026-localization-track-03', 'Track03', (0.15, 1.4))]):
    d = pd.read_csv(O + f'subs_{c}.csv')
    d = d[d.status.str.contains('COMPLETE') & d.publicScore.notna()].copy()
    d['t'] = pd.to_datetime(d.date) + pd.Timedelta(hours=8)
    d = d.sort_values('t')
    s = d.publicScore.astype(float).to_numpy()
    t = d.t.to_numpy()
    causal = d.fileName.str.contains('online|batch|ensemble|T3_sync').to_numpy()
    cut = np.datetime64('2026-10-22T18:00')
    ax.axvspan(t.min() - np.timedelta64(6, 'h'), cut, color='#eeeeee')
    ax.scatter(t[~causal], np.minimum(s[~causal], lim[1]), s=16, color='#a9b4c4')
    ax.scatter(t[causal], np.minimum(s[causal], lim[1]), s=18, color='#24364f')
    for m, col in ((~causal, '#a9b4c4'), (causal, '#24364f')):
        if m.any():
            ax.step(t[m], np.minimum.accumulate(s[m]), where='post', color=col, lw=2)
    for day, txt in [('2026-10-20 21:51', 'no board fitting'), ('2026-10-22 18:00', 'causal only')]:
        ax.axvline(pd.Timestamp(day), color='#24364f', ls='--', lw=1)
    ax.set_ylim(*lim); ax.set_title(lab); ax.set_ylabel('public score (lower = better)')
    ax.xaxis.set_major_formatter(mdates.DateFormatter('%m/%d'))
    ax.xaxis.set_major_locator(mdates.DayLocator(interval=2 if lab != 'Track03' else 1))
    if lab == 'Track03':
        ax.set_xlim(left=pd.Timestamp('2026-10-22 12:00'))
    ax.text(0.98, 0.95, 'final %.2f' % {'Track01': 0.45, 'Track02': 0.24, 'Track03': 0.19}[lab], transform=ax.transAxes, ha='right', va='top')
axs[0].text(0.02, 0.02, 'grey: not causal (RTS smoothing, board probing,\nmulti-pass) - not eligible; black: causal online', transform=axs[0].transAxes, fontsize=12)
plt.tight_layout(); plt.savefig(O + 'f_scores.png', dpi=110); plt.close()

# 2. MapAnything heading check (1.52 m camera height scale)
fr = ['600', '900', '1500', '0', '60', '120', '180', '240']
filt = [-0.38, -0.17, -0.08, -1.89, -0.87, -0.26, 0.00, 0.32]
icp = [3.46, -2.21, -0.04, -0.86, -1.19, -0.66, -0.48, 2.99]
x = np.arange(len(fr))
fig, ax = plt.subplots(figsize=(12, 5))
ax.bar(x - 0.2, filt, 0.4, color='#24364f', label='our filter')
ax.bar(x + 0.2, icp, 0.4, color='#8796ab', label='MapAnything cloud + ICP')
ax.axhline(0, color='#24364f', lw=0.8)
ax.set_xticks(x); ax.set_xticklabels([f'frame {f}' for f in fr], rotation=0, fontsize=12)
ax.axvline(2.5, color='#24364f', ls=':', lw=1)
ax.text(1, 3.9, 'moving, our pose already good', ha='center', fontsize=12); ax.text(5.5, 3.9, 'Track02 start (standing in a queue)', ha='center', fontsize=12)
ax.set_ylabel('heading error vs reference (deg)'); ax.set_ylim(-3, 4.4); ax.legend(loc='lower left', frameon=False)
plt.tight_layout(); plt.savefig(O + 'f_mapanything.png', dpi=110); plt.close()

# 3. T1 / T2 ensembles
fig, axs = plt.subplots(1, 2, figsize=(15, 5))
m1 = [('v7 config', 0.427), ('fwd_batch 8', 0.408), ('fwd_batch 10', 0.414), ('fwd_batch 12', 0.407), ('GNSS Huber', 0.409), ('MEAN', 0.404)]
m2 = [('v22', 0.201), ('search s4', 0.201), ('camera sync', 0.202), ('GNSS consider', 0.201), ('accumulation', 0.203), ('standstill\nGNSS bias', 0.223), ('MEAN', 0.191)]
for ax, m, t, lim in [(axs[0], m1, 'Track01: 5 causal runs, frame-wise mean\npublic 0.47 -> 0.45', (0.38, 0.44)), (axs[1], m2, 'Track02: 6 causal runs, frame-wise mean\npublic 0.25 -> 0.24', (0.18, 0.23))]:
    ax.bar(range(len(m)), [v for _, v in m], color=['#8796ab'] * (len(m) - 1) + ['#24364f'])
    ax.set_xticks(range(len(m))); ax.set_xticklabels([k for k, _ in m], fontsize=11, rotation=20)
    ax.set_ylim(*lim); ax.set_title(t, fontsize=14); ax.set_ylabel('error vs reference (score-like)')
    for i, (_, v) in enumerate(m):
        ax.text(i, v + 0.0008, f'{v:.3f}', ha='center', fontsize=11)
plt.tight_layout(); plt.savefig(O + 'f_ensembles.png', dpi=110); plt.close()

# 4. T3 start reference vs public
pts = [(0.338, 0.22), (0.329, 0.23), (0.342, 0.24), (0.383, 0.24), (0.385, 0.25), (0.405, 0.26), (0.568, 0.31), (0.823, 0.42)]
new = [(0.309, 0.20, 'kvo0 0.97'), (0.262, 0.19, 'posterior mean'), (0.247, 0.19, 'start lateral batch')]
fig, ax = plt.subplots(figsize=(9, 5.5))
ax.scatter([p[0] for p in pts], [p[1] for p in pts], color='#8796ab', s=60, label='earlier submissions (used to check it)')
for (e, s, l), ty in zip(new, (0.207, 0.188, 0.170)):
    ax.scatter(e, s, color='#24364f', s=70)
    ax.annotate(l, (e, s), xytext=(0.40, ty), textcoords='data', fontsize=12, arrowprops=dict(arrowstyle='-', lw=0.8))
ax.set_ylim(0.16, 0.44)
ax.set_xlabel('start error vs batch reference, frames 0-60 (m)'); ax.set_ylabel('Track03 public score')
ax.text(0.55, 0.36, 'rank correlation 0.92', fontsize=13)
ax.legend(loc='upper left', frameon=False, fontsize=12)
plt.tight_layout(); plt.savefig(O + 'f_t3start.png', dpi=110); plt.close()

# 5. pipeline
fig, ax = plt.subplots(figsize=(16, 6.5)); ax.axis('off')
boxes = [(0.02, 0.62, 'LiDAR map.pcd\n-> road-marking map'), (0.02, 0.18, 'camera images (3)\n-> road / vehicle masks'),
         (0.27, 0.62, 'BEV lane matching\n(terrain-aware IPM)'), (0.27, 0.18, 'road-surface VO\n(+ masked gap fill)'),
         (0.52, 0.40, 'joint EKF (causal)\nx, y, heading, speed,\naccel bias, VO scale,\ngyro bias'), (0.02, 0.40, 'IMU (+ GNSS T1/T2)'),
         (0.77, 0.62, 'T3: heading bank,\nposterior mean'), (0.77, 0.18, 'T1/T2: mean of\n5-6 causal runs'), (0.77, 0.40, 'idx, x, y, yaw\nper frame')]
for x, y, t in boxes:
    ax.add_patch(plt.Rectangle((x, y), 0.2, 0.18, fill=False, lw=1.5, transform=ax.transAxes))
    ax.text(x + 0.1, y + 0.09, t, ha='center', va='center', fontsize=13, transform=ax.transAxes)
for (x0, y0), (x1, y1) in [((0.22, 0.71), (0.27, 0.71)), ((0.22, 0.27), (0.27, 0.27)), ((0.47, 0.71), (0.52, 0.55)), ((0.47, 0.27), (0.52, 0.43)),
                           ((0.22, 0.49), (0.52, 0.49)), ((0.72, 0.53), (0.77, 0.68)), ((0.72, 0.43), (0.77, 0.27)), ((0.72, 0.49), (0.77, 0.49))]:
    ax.annotate('', xy=(x1, y1), xytext=(x0, y0), xycoords='axes fraction', arrowprops=dict(arrowstyle='->', lw=1.4))
plt.savefig(O + 'f_pipeline.png', dpi=110, bbox_inches='tight'); plt.close()
print('ok')
