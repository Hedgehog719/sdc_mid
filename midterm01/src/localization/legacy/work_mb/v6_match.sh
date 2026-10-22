#!/bin/bash
# v6 lane fixes: road-voted marking maps (camera road segmentation) + near-range weighting (--range_weight 10)
set -e
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_vert.json
L=work/mb/v6; mkdir -p $L
T2P=submissions/day7/track2/t2_pipe_v5.csv
T1P=submissions/day7/track1/t1_pipe_v5.csv
[ -f work/mk_c16p20_10v_S.npz ] || python3 tools/marking_road_vote.py /root/data/Track2 $T2P work/mk_c16p20_10_S.npz work/mk_c16p20_10v_S.npz --thr 0.02 > $L/vote_p20.log 2>&1
BMOPT="--accum 20 --accum_step 4 --workers 4 --yaw_mount -1.5 --percam --accum_dist 10 --accum_ds 0.7 --pitch_per_accel -0.8 --dyn_attitude --dyn_roll_only --dyn_sign 1 --ground_mode profile --yaw_fine 0.05 --range_weight 10"
echo "[$(date +%H:%M:%S)] T2 start"
SDC_MAP_TAG=c16p20_10 python3 tools/bev_match.py /root/data/Track2 $T2P work/v6s_bm_t2.npz --frames 0:450:3 $BMOPT --map_marks work/mk_c16p20_10v_S.npz --max_yaw 6 > $L/v6s_t2.log 2>&1
echo "[$(date +%H:%M:%S)] T2 rest"
SDC_MAP_TAG=c16p20_10 python3 tools/bev_match.py /root/data/Track2 $T2P work/v6r_bm_t2.npz --frames 450:3299:3 $BMOPT --map_marks work/mk_c16p20_10v_S.npz > $L/v6r_t2.log 2>&1
echo "[$(date +%H:%M:%S)] T1 full"
SDC_MAP_TAG=clean15 python3 tools/bev_match.py /root/data/Track1 $T1P work/v6_bm_t1.npz --frames 0:3011:3 $BMOPT --map_marks work/mk1_15v_S.npz > $L/v6_t1.log 2>&1
python3 - <<'PY'
import numpy as np
a, b = dict(np.load('work/v6s_bm_t2.npz')), dict(np.load('work/v6r_bm_t2.npz'))
out = {k: np.concatenate([a[k], b[k]]) for k in a if k in b}
o = np.argsort(out['idx']); np.savez('work/v6_bm_t2m.npz', **{k: v[o] for k, v in out.items()}); print('T2 merged', len(o))
PY
chown -R 1000:1000 work/v6* work/mk_c16p20_10v_S.npz $L
echo "[$(date +%H:%M:%S)] done"
