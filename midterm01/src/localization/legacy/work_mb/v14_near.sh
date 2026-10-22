#!/bin/bash
# T2 near fixes with background penalty (marks - background, as T1); start 0-330 front camera on the plate map as nf11g
set -e
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_v10t2.json
python3 tools/near_fix.py /root/data/Track2 work/mk16v_D.npz submissions/day7/track2/t2_pipe_v5.csv --frames 330:-1:3 --workers 2 --bg_weight 1.0 --yaw_search 1.0 --out work/nf14r_t2.npz 2>&1 | tail -1
python3 tools/near_fix.py /root/data/Track2 work/mk_c16p20_10v_D.npz submissions/day7/track2/t2_pipe_v5.csv --frames 0:330:3 --workers 2 --bg_weight 1.0 --yaw_search 1.0 --cams f --out work/nf14s_t2.npz 2>&1 | tail -1
python3 - <<'PY'
import numpy as np
a = dict(np.load('work/nf14s_t2.npz')); b = dict(np.load('work/nf14r_t2.npz'))
out = {k: np.concatenate([a[k], b[k]]) for k in b if k in a}
o = np.argsort(out['idx']); np.savez('work/nf14_t2.npz', **{k: v[o] for k, v in out.items()}); print('merged', len(o))
PY
C2="--yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --pole work/pc_pm_t2.npz --pole_pct 60 --pole_lat_sigma 0.6 --pole_fwd_sigma 0.25 --road work/c_rd_t2.npz --road_pct 50 --road_lat_sigma 0.5 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 2.0 --yaw_rts_base 2 --near_yaw_sigma 0.3 --gnss_bias_tau 20 --gnss_white 0.08 --gnss_bias_sigma 0.07 --map_bias_len 1000 --map_bias_sigma 0.05 --map_bias_sigma_lat 0.02 --bev_yaw_skip 0:330 --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c16_p1.csv --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.15 --near_aniso_k 0.004 --bev_fwd_skip 0:330 --bev work/vert_bm_t2m.npz --yaw_selfcal --near_yaw_skip 0:330"
mkdir -p work/n14
python3 tools/fuse.py /root/data/Track2 work/n14/nf14.csv $C2 --near work/nf14_t2.npz > /dev/null 2>&1 &
python3 tools/fuse.py /root/data/Track2 work/n14/nf11g.csv $C2 --near work/nf11g_t2.npz > /dev/null 2>&1 &
python3 tools/fuse.py /root/data/Track2 work/n14/nf14_lat0.10.csv $C2 --near work/nf14_t2.npz --near_lat_sigma 0.10 > /dev/null 2>&1 &
python3 tools/fuse.py /root/data/Track2 work/n14/nf14_k0.003.csv $C2 --near work/nf14_t2.npz --near_aniso_k 0.003 > /dev/null 2>&1 &
wait
chown -R 1000:1000 work/n14 work/nf14*
python3 work/mb/est_score.py T2 $(ls work/n14/*.csv) 2>&1 | grep "estimated\|after 330" | paste - - | sed 's/(in span [0-9]* %)//g; s/\[current best 0.17429\]//; s/   vs best, after 330   ://' | awk '{print $1, $4, $7, $13, $14, $15, $16, $17, $18, $19, $20, $21, $22}' | sort -k2 -n
