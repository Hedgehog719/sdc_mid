# more figures for the report deck: early history, cost of the rules, causal steps
import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 15, 'axes.spines.top': False, 'axes.spines.right': False, 'figure.facecolor': 'white'})
O = 'work/deck/'

# 1. before the rules: from GNSS to the board optimum (public, best so far)
T1 = [('10/16', 1.31, 'GNSS only'), ('10/16', 1.08, 'GNSS + IMU heading, RTS'), ('10/17', 1.00, '+ BEV lane lateral fixes'),
      ('10/18', 0.93, ''), ('10/19', 0.70, ''), ('10/19', 0.52, 'LiDAR marking map'), ('10/19', 0.50, ''),
      ('10/20', 0.41, 'board optimum (probing)')]
T2 = [('10/16', 0.54, 'GNSS + IMU heading'), ('10/16', 0.34, 'motion heading, RTS'), ('10/18', 0.28, 'lane heading fixes'),
      ('10/19', 0.22, 'heading = course (4 s window)'), ('10/19', 0.21, ''), ('10/20', 0.17, 'board optimum (probing)')]
fig, axs = plt.subplots(1, 2, figsize=(16, 5.6))
for ax, (name, seq) in zip(axs, [('Track01', T1), ('Track02', T2)]):
    y = [s for _, s, _ in seq]
    x = np.arange(len(y))
    ax.step(x[:-1], y[:-1], where='post', color='#24364f', lw=2)
    ax.plot(x[-2:], y[-2:], color='#8796ab', lw=2, ls='--')
    ax.scatter(x[:-1], y[:-1], color='#24364f', zorder=3)
    ax.scatter(x[-1:], y[-1:], color='#8796ab', zorder=3)
    for xi, (d, s, lab) in zip(x, seq):
        if lab:
            ax.annotate(f'{s:.2f}  {lab}', (xi, s), xytext=(8, 6), textcoords='offset points', fontsize=13,
                        color='#b0643c' if 'probing' in lab else '#24364f')
    ax.set_xticks(x); ax.set_xticklabels([d for d, _, _ in seq], rotation=40, fontsize=12)
    ax.set_xlim(-0.4, len(y) + 2.6)
    ax.set_title(name); ax.set_ylabel('public score (best so far)')
plt.tight_layout(); plt.savefig(O + 'f_early.png', dpi=110); plt.close()

# 2. what going causal cost, and what came back
groups = ['Track01', 'Track02', 'Track03']
rts = [0.40, 0.17, np.nan]
first = [0.48, 0.34, 0.86]
final = [0.45, 0.24, 0.19]
fig, ax = plt.subplots(figsize=(11, 5.6))
w = 0.26
x = np.arange(3)
for k, (vals, col, lab) in enumerate([(rts, '#c9d1dc', 'RTS smoothing, 10/21 (not allowed)'),
                                      (first, '#b0643c', 'first causal filter, 10/22'),
                                      (final, '#24364f', 'final causal, 10/28')]):
    b = ax.bar(x + (k - 1) * w, vals, w, color=col, label=lab)
    for xi, v in zip(x + (k - 1) * w, vals):
        if not np.isnan(v):
            ax.text(xi, v + 0.012, f'{v:.2f}', ha='center', fontsize=13)
ax.text(2 - w, 0.03, 'no\nGNSS', ha='center', fontsize=12, color='#b0643c')
ax.set_xticks(x); ax.set_xticklabels(groups); ax.set_ylabel('public score')
ax.legend(frameon=False, loc='upper left')
plt.tight_layout(); plt.savefig(O + 'f_rules.png', dpi=110); plt.close()


def steps(ax, title, seq):
    labs = [l for l, _ in seq][::-1]
    vals = [v for _, v in seq][::-1]
    cols = ['#24364f' if i == 0 else '#8796ab' for i in range(len(vals))]
    ax.barh(range(len(vals)), vals, color=cols, height=0.62)
    for i, v in enumerate(vals):
        ax.text(v + 0.01, i, f'{v:.2f}', va='center', fontsize=13)
    ax.set_yticks(range(len(vals))); ax.set_yticklabels(labs, fontsize=13)
    ax.set_xlim(0, max(vals) * 1.15); ax.set_xlabel('public score')
    ax.set_title(title)


# 3. Track03 causal steps (no GNSS)
fig, ax = plt.subplots(figsize=(11, 6.2))
steps(ax, 'Track03, causal versions', [
    ('first causal filter (10/22)', 0.86), ('heading hypothesis bank', 0.51), ('terrain-aware BEV', 0.37),
    ('fast pitch + body roll', 0.25), ('lateral road shape', 0.24), ('soft heading peak, camera sync', 0.22),
    ('initial VO scale 0.97', 0.20), ('posterior mean of the bank', 0.19), ('+ start lateral batch (final)', 0.19)])
plt.tight_layout(); plt.savefig(O + 'f_steps_t3.png', dpi=110); plt.close()

# 4. Track01 / Track02 causal steps
fig, axs = plt.subplots(1, 2, figsize=(17, 5.6))
steps(axs[0], 'Track01, causal versions', [
    ('first causal filter (10/22)', 0.48), ('IMU only up to the frame time', 0.46), ('braking pitch model', 0.46),
    ('mean of 5 runs (final)', 0.45)])
steps(axs[1], 'Track02, causal versions', [
    ('first causal filter (10/22)', 0.34), ('standstill heading re-search', 0.30), ('re-search sigma 2.5', 0.28),
    ('lanes only on visible road', 0.27), ('lane heading sigma 2.5', 0.26), ('mean of 5 variants', 0.25),
    ('+ standstill GNSS bias run (final)', 0.24)])
plt.tight_layout(); plt.savefig(O + 'f_steps_t12.png', dpi=110); plt.close()
