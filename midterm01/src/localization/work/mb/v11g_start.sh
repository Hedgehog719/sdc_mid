#!/bin/bash
# v11g: T2 start (0-330) near fixes from the front camera only (side cameras see parked cars leaking into the marking
# map there; per-camera segment curves: f +0.1, fl -0.15, fr -0.55 on v11f), lane fixes lateral-only over 0:330
set -e
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_v10t2.json
python3 tools/near_fix.py /root/data/Track2 work/mk_c16p20_10v_D.npz submissions/day7/track2/t2_pipe_v5.csv --frames 0:330:3 --workers 4 --yaw_search 1.0 --cams f --out work/nf11gs_t2.npz 2>&1 | tail -1
python3 - <<'PY'
import numpy as np
a = dict(np.load('work/nf11gs_t2.npz')); b = dict(np.load('work/nf11_t2.npz'))
keep = b['idx'] >= 330
out = {k: np.concatenate([a[k], b[k][keep]]) for k in b if k in a}
o = np.argsort(out['idx']); np.savez('work/nf11g_t2.npz', **{k: v[o] for k, v in out.items()})
PY
C2="--yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --pole work/pc_pm_t2.npz --pole_pct 60 --pole_fwd_sigma 0.4 --pole_lat_sigma 0.6 --road work/c_rd_t2.npz --road_pct 50 --road_lat_sigma 0.5 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 0.7 --gnss_bias_tau 20 --gnss_bias_sigma 0.14 --gnss_white 0.03 --bev_yaw_skip 0:330 --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c16_p1.csv --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.15 --near_aniso_k 0.004 --near_yaw_sigma 0.3 --bev work/v10g_bm_t2m.npz"
python3 tools/fuse.py /root/data/Track2 work/_v11g_t2.csv $C2 --near work/nf11g_t2.npz --bev_fwd_skip 0:330 --nis 2>&1 | grep "NIS near" | cut -c1-80
chown 1000:1000 work/nf11g* work/_v11g_t2.csv
python3 - <<'PY'
import numpy as np, pandas as pd
A = pd.read_csv('work/_v10_t2.csv'); yw = A.yaw.values
for f in ('work/_v11f_t2.csv', 'work/_v11g_t2.csv'):
    B = pd.read_csv(f); dx, dy = B.x.values - A.x.values, B.y.values - A.y.values
    fw, lt = np.cos(yw) * dx + np.sin(yw) * dy, -np.sin(yw) * dx + np.cos(yw) * dy
    print(f'{f} - v10 fwd/lat: ' + ' | '.join(f'{s0}-{s1}: {fw[s0:s1].mean():+.3f}/{lt[s0:s1].mean():+.3f}' for s0, s1 in ((0, 150), (150, 330), (330, 450), (450, 3300))))
PY
python3 work/mb/est_score.py T2 work/_v10_t2.csv work/_v11g_t2.csv | grep 'estimated\|start'
for fr in 0:150:3 150:330:3; do python3 work/mb/seg_fwd_curve.py /root/data/Track2 work/mk_c16p20_10v_D.npz work/_v11g_t2.csv $fr 2>&1 | grep -v Warn | sed 's/  curve.*//' | grep -v "marks  "; done
python3 tools/render_paint.py /root/data/Track2 work/_v11g_t2.csv work/mk_c16p20_10v_D.npz --frames 0:450:3 --name start_v11g > /dev/null 2>&1
chown -R 1000:1000 work/viewer
