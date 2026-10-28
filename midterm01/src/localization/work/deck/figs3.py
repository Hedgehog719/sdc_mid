# vertical pipeline figure, large text
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
steps = ['Inputs: 3 cameras, IMU, GNSS (T1/T2), LiDAR map.pcd',
         'Preprocess per image: road / vehicle masks,\nmarking response; map -> marking map',
         'Frame 0: position from init_guess / GNSS,\nheading search +-25 deg on lane match',
         'Every frame: IMU predict -> VO speed -> GNSS',
         'Every 3 frames: BEV lane match + near alignment\n(terrain-aware for T3); standstill: ZUPT',
         'Write pose (never changed later)',
         'T3: 5 heading hypotheses, posterior mean\nT1/T2: mean of 5 / 6 causal filters']
fig, ax = plt.subplots(figsize=(10, 12)); ax.axis('off')
n = len(steps); h = 1 / n
for i, t in enumerate(steps):
    y = 1 - (i + 1) * h + 0.02
    ax.add_patch(plt.Rectangle((0.03, y), 0.94, h - 0.045, fill=True, fc='#eef1f5', ec='#24364f', lw=1.5, transform=ax.transAxes))
    ax.text(0.5, y + (h - 0.045) / 2, t, ha='center', va='center', fontsize=19, color='#1a1a1a', transform=ax.transAxes)
    if i < n - 1:
        ax.annotate('', xy=(0.5, y - 0.022), xytext=(0.5, y), xycoords='axes fraction', arrowprops=dict(arrowstyle='->', lw=2, color='#24364f'))
plt.savefig('work/deck/f_pipeline_v.png', dpi=110, bbox_inches='tight'); plt.close()
