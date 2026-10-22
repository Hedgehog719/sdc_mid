#!/bin/bash
# lane fixes with the re-calibrated side cameras (cam_fix_v8): T2 as v5 (plate map, unvoted soft marks), T1 as v6b (voted)
set -e
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_v9.json
L=work/mb/v9; mkdir -p $L
BMOPT="--accum 20 --accum_step 4 --workers 4 --yaw_mount -1.5 --percam --accum_dist 10 --accum_ds 0.7 --pitch_per_accel -0.8 --dyn_attitude --dyn_roll_only --dyn_sign 1 --ground_mode profile --yaw_fine 0.05"
T2TR=submissions/day2/track2/yaw_traj_q0.05_bev.csv
echo "[$(date +%H:%M:%S)] T2 start"
SDC_MAP_TAG=c16p20_10 python3 tools/bev_match.py /root/data/Track2 $T2TR work/v9s_bm_t2.npz --frames 0:450:3 $BMOPT --map_marks work/mk_c16p20_10_S.npz --max_yaw 6 > $L/s_t2.log 2>&1
SDC_MAP_TAG=c16p20_10 python3 tools/bev_match.py /root/data/Track2 $T2TR work/v9r_bm_t2.npz --frames 450:3299:3 $BMOPT --map_marks work/mk_c16p20_10_S.npz > $L/r_t2.log 2>&1
python3 - <<'PY'
import numpy as np
a, b = dict(np.load('work/v9s_bm_t2.npz')), dict(np.load('work/v9r_bm_t2.npz'))
out = {k: np.concatenate([a[k], b[k]]) for k in a if k in b}
o = np.argsort(out['idx']); np.savez('work/v9_bm_t2m.npz', **{k: v[o] for k, v in out.items()}); print('T2 merged', len(o))
PY
echo "[$(date +%H:%M:%S)] T1"
SDC_MAP_TAG=clean15 python3 tools/bev_match.py /root/data/Track1 submissions/day7/track1/t1_pipe_v5.csv work/v9_bm_t1.npz --frames 0:3011:3 $BMOPT --map_marks work/mk1_15v_S.npz > $L/t1.log 2>&1
chown -R 1000:1000 work/v9* $L
echo "[$(date +%H:%M:%S)] done"
