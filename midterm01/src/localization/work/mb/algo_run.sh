#!/bin/bash
# Score-free run on the repaired maps (T2 map_clean16, T1 map_clean15): marking maps -> lane matching (vertical-only
# camera calibration) -> two-pass RTS fusion (GNSS + IMU + lane / pole / kerb fixes). No leaderboard-derived numbers:
# no blend weights, no probe offsets (yaw_bias / xy_bias / time_shift = 0), no board-derived start.
# Priors (search window centres only) are the earlier algorithmic trajectories.
set -e
cd /root/midterm/src/localization
T2TR=submissions/day2/track2/yaw_traj_q0.05_bev.csv
T1TR=submissions/day3/track1/iter4_lane_pole_curb_hold.csv
L=work/mb/algo
mkdir -p $L
step() { echo "[$(date +%H:%M:%S)] $*"; }

step marking maps
[ -f work/mk16_S.npz ] || SDC_MAP_TAG=clean16 python3 tools/marking_map.py /root/data/Track2 $T2TR work/mk16_S.npz --mode soft > $L/mk16_S.log 2>&1
[ -f work/mk16_D.npz ] || SDC_MAP_TAG=clean16 python3 tools/marking_map.py /root/data/Track2 $T2TR work/mk16_D.npz --hi 2.5 --lo 1.2 --cell 0.7 > $L/mk16_D.log 2>&1
[ -f work/mk1_15_S.npz ] || SDC_MAP_TAG=clean15 python3 tools/marking_map.py /root/data/Track1 $T1TR work/mk1_15_S.npz --mode soft > $L/mk1_15_S.log 2>&1

BMOPT="--accum 20 --accum_step 4 --workers 4 --yaw_mount -1.5 --percam --accum_dist 10 --accum_ds 0.7 --pitch_per_accel -0.8 --dyn_attitude --dyn_roll_only --dyn_sign 1 --ground_mode profile --yaw_fine 0.05"
step T2 lane matching, start segment +-6 deg
[ -f work/c16s_bm_t2.npz ] || SDC_ROT_FIX=work/cam_fix_vert.json SDC_MAP_TAG=clean16 python3 tools/bev_match.py /root/data/Track2 $T2TR work/c16s_bm_t2.npz --frames 0:450:3 $BMOPT --map_marks work/mk16_S.npz --max_yaw 6 > $L/c16s_bm_t2.log 2>&1
step T2 lane matching, full
[ -f work/c16_bm_t2.npz ] || SDC_ROT_FIX=work/cam_fix_vert.json SDC_MAP_TAG=clean16 python3 tools/bev_match.py /root/data/Track2 $T2TR work/c16_bm_t2.npz --frames 0:3299:3 $BMOPT --map_marks work/mk16_S.npz > $L/c16_bm_t2.log 2>&1
step T1 lane matching, full
[ -f work/c15_bm_t1.npz ] || SDC_ROT_FIX=work/cam_fix_vert.json SDC_MAP_TAG=clean15 python3 tools/bev_match.py /root/data/Track1 $T1TR work/c15_bm_t1.npz --frames 0:3011:3 $BMOPT --map_marks work/mk1_15_S.npz > $L/c15_bm_t1.log 2>&1

step T2 fusion
python3 - <<'PY'
import numpy as np
a = dict(np.load('work/c16_bm_t2.npz')); b = dict(np.load('work/c16s_bm_t2.npz'))
keep = a['idx'] >= 450
out = {k: np.concatenate([b[k], a[k][keep]]) for k in a if k in b}
o = np.argsort(out['idx']); out = {k: v[o] for k, v in out.items()}
np.savez('work/c16_bm_t2m.npz', **out); print('T2 merged fixes', len(out['idx']))
PY
C2="--yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 --rts --bev work/c16_bm_t2m.npz --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --pole work/pc_pm_t2.npz --pole_pct 60 --pole_fwd_sigma 0.4 --pole_lat_sigma 0.6 --road work/c_rd_t2.npz --road_pct 50 --road_lat_sigma 0.5 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 0.7 --gnss_bias_tau 10 --gnss_bias_sigma 0.2 --gnss_white 0.03"
python3 tools/fuse.py /root/data/Track2 work/_c16_p1.csv $C2 --rts_sigma 0.3 --rts_qpos 0.4 --rts_qlat 0.1 > $L/fuse_t2_p1.log 2>&1
python3 tools/fuse.py /root/data/Track2 work/_c16_gm.csv $C2 --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c16_p1.csv --nis > $L/fuse_t2.log 2>&1

step T1 fusion
B1="--yaw_src imu_cf --yaw_cf_sigma 10 --rts --rts_qacc 1.0 --bev work/c15_bm_t1.npz --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --yaw_rts traj --yaw_rts_q 0.1 --yaw_rts_bev 1.0 --road work/c6_rds_t1.npz --road_pct 50 --road_lat_sigma 0.5 --start_hold 1.5 --gnss_bias_tau 20 --gnss_bias_sigma 0.25 --gnss_white 0.02 --map_bias_len 1000 --map_bias_sigma 0.03"
python3 tools/fuse.py /root/data/Track1 work/_c15_p1.csv $B1 --rts_sigma 0.5 --rts_qpos 0.4 --rts_qlat 0.1 > $L/fuse_t1_p1.log 2>&1
P2="--rts_sigma 0.5 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c15_p1.csv"
python3 tools/fuse.py /root/data/Track1 work/_c15_np.csv $B1 $P2 > $L/fuse_t1_np.log 2>&1
python3 - <<'PY'
import numpy as np, pandas as pd
T = pd.read_csv('work/_c15_np.csv'); z = dict(np.load('work/c6_pm_t1.npz')); i = z['idx'].astype(int)
yw = T.yaw.values[i]; dx, dy = z['mx'] - T.x.values[i], z['my'] - T.y.values[i]
fw, lt = np.cos(yw) * dx + np.sin(yw) * dy, -np.sin(yw) * dx + np.cos(yw) * dy
k = (np.abs(fw) < 0.8) & (np.abs(lt) < 0.6)
np.savez('work/_c15_pg.npz', **{kk: v[k] for kk, v in z.items()}); print('T1 gated poles', k.sum(), '/', len(k))
PY
python3 tools/fuse.py /root/data/Track1 work/_c15_v2.csv $B1 $P2 --pole work/_c15_pg.npz --pole_pct 50 --pole_fwd_sigma 0.4 --pole_lat_sigma 0.6 --nis > $L/fuse_t1.log 2>&1

mkdir -p submissions/day7/track1 submissions/day7/track2
cp work/_c16_gm.csv submissions/day7/track2/t2_algo_c16.csv
cp work/_c15_v2.csv submissions/day7/track1/t1_algo_c15.csv
chown -R 1000:1000 submissions/day7 work/_c16_* work/_c15_* work/c16* work/c15* work/mk16_* work/mk1_15_* $L
step done
