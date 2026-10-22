#!/bin/bash
set -e
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_v8.json
L=work/mb/fr; mkdir -p $L
BMOPT="--accum 20 --accum_step 4 --workers 4 --yaw_mount -1.5 --percam --accum_dist 10 --accum_ds 0.7 --pitch_per_accel -0.8 --dyn_attitude --dyn_roll_only --dyn_sign 1 --ground_mode profile --yaw_fine 0.05"
# T1: lane fixes re-matched around the v8 trajectory (better prior: centring + accumulation)
( SDC_MAP_TAG=clean15 python3 tools/bev_match.py /root/data/Track1 work/_v8_t1.csv work/fr_bm_t1.npz --frames 0:3011:3 $BMOPT --map_marks work/mk1_15v_S.npz > $L/t1.log 2>&1; echo T1 done >> $L/status ) &
# T2: start plate shift 0.25 m (s < 20 m, taper 10 m)
T2TR=submissions/day2/track2/yaw_traj_q0.05_bev.csv
[ -f /root/data/Track2/map_c16p25.npy ] || python3 tools/splice_level.py /root/data/Track2 $T2TR --tag_in clean16 --tag_out c16p25 --max_dz 0 --shift_fwd 0.25 --hseam 20 --hband 10 > /dev/null
SDC_MAP_TAG=c16p25 python3 tools/marking_map.py /root/data/Track2 $T2TR work/mk_c16p25_S.npz --mode soft > /dev/null
SDC_MAP_TAG=c16p25 python3 tools/marking_map.py /root/data/Track2 $T2TR work/mk_c16p25_D.npz --hi 2.5 --lo 1.2 --cell 0.7 > /dev/null
python3 tools/marking_road_vote.py /root/data/Track2 work/_v8_t2.csv work/mk_c16p25_D.npz work/mk_c16p25v_D.npz --thr 0.5 | tail -1
SDC_MAP_TAG=c16p25 python3 tools/bev_match.py /root/data/Track2 $T2TR work/frs_bm_t2.npz --frames 0:450:3 $BMOPT --map_marks work/mk_c16p25_S.npz --max_yaw 6 > $L/t2s.log 2>&1
python3 -c "
import numpy as np
for f in ('work/v8s_bm_t2.npz', 'work/frs_bm_t2.npz'):
    z = np.load(f); print(f, 'start median peak %.4f' % np.median(z['peak']))"
python3 tools/near_fix.py /root/data/Track2 work/mk_c16p25v_D.npz submissions/day7/track2/t2_pipe_v5.csv --frames 0:450:3 --workers 4 --out work/nf25s_t2.npz | tail -1
python3 - <<'PY'
import numpy as np
a = dict(np.load('work/frs_bm_t2.npz')); b = dict(np.load('work/v8_bm_t2m.npz'))
keep = b['idx'] >= 450
out = {k: np.concatenate([a[k], b[k][keep]]) for k in b if k in a}
o = np.argsort(out['idx']); np.savez('work/fr_bm_t2m.npz', **{k: v[o] for k, v in out.items()})
a = dict(np.load('work/nf25s_t2.npz')); b = dict(np.load('work/nf8_t2.npz'))
keep = b['idx'] >= 450
out = {k: np.concatenate([a[k], b[k][keep]]) for k in b if k in a}
o = np.argsort(out['idx']); np.savez('work/nf25_t2.npz', **{k: v[o] for k, v in out.items()})
z = dict(np.load('work/nf25s_t2.npz')); i = z['idx']
for s0, s1 in ((0, 250), (250, 450)):
    m = (i >= s0) & (i < s1); print(f'p25 start near fixes {s0}-{s1}: n {m.sum()} fwd {np.median(z["fwd"][m]):+.3f} lat {np.median(z["lat"][m]):+.3f}')
PY
wait
chown -R 1000:1000 work/fr_* work/frs_* work/nf25* work/mk_c16p25* $L
echo final round done
