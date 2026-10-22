#!/bin/bash
# v11 step 2: during the (near-)stationary T2 start the lane fixes give lateral only (--bev_fwd_skip), forward from the
# near-range fixes on the same start map; plus T1 start residual check
set -e
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_v10t2.json
C2="--yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --pole work/pc_pm_t2.npz --pole_pct 60 --pole_fwd_sigma 0.4 --pole_lat_sigma 0.6 --road work/c_rd_t2.npz --road_pct 50 --road_lat_sigma 0.5 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 0.7 --gnss_bias_tau 20 --gnss_bias_sigma 0.14 --gnss_white 0.03 --bev_yaw_skip 0:330 --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c16_p1.csv --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.15 --near_aniso_k 0.004 --near_yaw_sigma 0.3 --bev work/v10g_bm_t2m.npz --near work/nf11_t2.npz"
python3 tools/fuse.py /root/data/Track2 work/_v11f_t2.csv $C2 --bev_fwd_skip 0:330 --nis 2>&1 | grep "NIS" | cut -c1-80
chown 1000:1000 work/_v11f_t2.csv
python3 - <<'PY'
import numpy as np, pandas as pd
def res(F, z, name, segs):
    i = z['idx'].astype(int); yw = F.yaw.values[i]
    dx, dy = z['mx'] - F.x.values[i], z['my'] - F.y.values[i]
    fw, lt = np.cos(yw)*dx+np.sin(yw)*dy, -np.sin(yw)*dx+np.cos(yw)*dy
    print(f'  {name:5s} ' + ' | '.join(f'{s0}-{s1}: {np.median(fw[(i>=s0)&(i<s1)]):+.2f}/{np.median(lt[(i>=s0)&(i<s1)]):+.2f}' for s0, s1 in segs if ((i>=s0)&(i<s1)).sum()))
S = ((0, 150), (150, 330), (330, 450), (450, 900))
for f in ('work/_v10_t2.csv', 'work/_v11_t2.csv', 'work/_v11f_t2.csv'):
    F = pd.read_csv(f); print(f, 'fix - fused (fwd/lat)')
    res(F, dict(np.load('work/nf11_t2.npz')), 'near', S); res(F, dict(np.load('work/v10g_bm_t2m.npz')), 'lane', S)
F = pd.read_csv('work/_v10_t1.csv'); sp = np.hypot(np.diff(F.x), np.diff(F.y))
print('T1 v10 fix - fused; speed m/frame ' + ' '.join(f'{s0}:{sp[s0:s0+110].mean():.3f}' for s0 in (0, 110, 220, 330)))
S1 = ((0, 110), (110, 220), (220, 330), (330, 450))
res(F, dict(np.load('work/nf10_t1.npz')), 'near', S1); res(F, dict(np.load('work/v10g_bm_t1.npz')), 'lane', S1)
PY
python3 work/mb/est_score.py T2 work/_v10_t2.csv work/_v11_t2.csv work/_v11f_t2.csv | grep 'estimated\|start\|after'
for f in work/_v10_t2.csv work/_v11f_t2.csv; do echo "percam $f"; python3 work/mb/percam_align.py /root/data/Track2 work/mk_c16p20_10v_D.npz $f 0:900:9 2>&1 | grep -E '^ +(0|300|600):'; done
