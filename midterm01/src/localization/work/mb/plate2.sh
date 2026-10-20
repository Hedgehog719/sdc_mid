#!/bin/bash
# shorter start plate: shift 0.55 full up to s=HS, linear to 0 over HB; start fixes 0:700 rematched
set -e
cd /root/midterm/src/localization
T2TR=submissions/day2/track2/yaw_traj_q0.05_bev.csv
BMOPT="--accum 20 --accum_step 4 --workers 4 --yaw_mount -1.5 --percam --accum_dist 10 --accum_ds 0.7 --pitch_per_accel -0.8 --dyn_attitude --dyn_roll_only --dyn_sign 1 --ground_mode profile --yaw_fine 0.05"
for v in "10 10" "20 10" "8 6"; do set -- $v; HS=$1; HB=$2; T=c16p${HS}_${HB}
  [ -f /root/data/Track2/map_$T.npy ] || python3 tools/splice_level.py /root/data/Track2 $T2TR --tag_in clean16 --tag_out $T --max_dz 0 --shift_fwd 0.55 --hseam $HS --hband $HB > /dev/null
  [ -f work/mk_${T}_S.npz ] || SDC_MAP_TAG=$T python3 tools/marking_map.py /root/data/Track2 $T2TR work/mk_${T}_S.npz --mode soft > /dev/null
  [ -f work/${T}a_bm_t2.npz ] || SDC_ROT_FIX=work/cam_fix_vert.json SDC_MAP_TAG=$T python3 tools/bev_match.py /root/data/Track2 $T2TR work/${T}a_bm_t2.npz --frames 0:450:3 $BMOPT --map_marks work/mk_${T}_S.npz --max_yaw 6 > /dev/null 2>&1
  [ -f work/${T}b_bm_t2.npz ] || SDC_ROT_FIX=work/cam_fix_vert.json SDC_MAP_TAG=$T python3 tools/bev_match.py /root/data/Track2 $T2TR work/${T}b_bm_t2.npz --frames 450:700:3 $BMOPT --map_marks work/mk_${T}_S.npz > /dev/null 2>&1
  python3 - $T <<'PY'
import sys, numpy as np
T = sys.argv[1]
a, b, c = (dict(np.load(f)) for f in (f'work/{T}a_bm_t2.npz', f'work/{T}b_bm_t2.npz', 'work/c16_bm_t2.npz'))
print(T, 'start peak %.4f  450-700 peak %.4f' % (np.median(a['peak']), np.median(b['peak'])))
keep = c['idx'] >= 700
out = {k: np.concatenate([a[k], b[k], c[k][keep]]) for k in c if k in a and k in b}
o = np.argsort(out['idx']); np.savez(f'work/{T}_bm_t2m.npz', **{k: v[o] for k, v in out.items()})
PY
done
echo plate2 done
