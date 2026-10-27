#!/bin/bash
# reproducibility check (validation): rebuild a track's runner intermediates from the raw data with the runner's commands
# and environment into work/rebuild/<TR>, then compare them array by array with the caches in work/runs/<TR>
# usage: rebuild_check.sh <T1|T2|T3>
cd /root/midterm/src/localization
TR=$1; TD=/root/data/Track${TR#T}; O=work/rebuild/$TR; mkdir -p $O
case $TR in
  T1) export SDC_ROT_FIX=calib/cam_fix_v10t1.json ;;
  T2) export SDC_ROT_FIX=calib/cam_fix_v10t2.json ;;
  T3) export SDC_ROT_FIX=calib/cam_fix_v10a.json SDC_CAM_SYNC=1 SDC_CAM_SYNC_YAW=1 ;;
esac
python3 tools/marking_map.py "$TD" ALL $O/mk_D.npz --hi 2.5 --lo 1.2 --cell 0.7 > $O/mk_D.log 2>&1 &
python3 tools/marking_map.py "$TD" ALL $O/mk_S.npz --mode soft > $O/mk_S.log 2>&1 &
python3 tools/bev_vo.py "$TD" $O/vo.npz > $O/vo.log 2>&1 &
wait
if [ $TR = T3 ]; then
  python3 tools/bev_vo.py "$TD" $O/vo_m30.npz --mask_tol 0.3 > $O/vo_m30.log 2>&1
  python3 tools/vo_fill.py $O/vo.npz $O/vo_m30.npz $O/vo_fill.npz > $O/vo_fill.log 2>&1
fi
python3 - <<PY > $O/compare.txt 2>&1
import numpy as np, os
for f in ['mk_D.npz', 'mk_S.npz', 'vo.npz', 'vo_m30.npz', 'vo_fill.npz']:
    a, b = 'work/rebuild/$TR/' + f, 'work/runs/$TR/' + f
    if not os.path.exists(a):
        continue
    za, zb = np.load(a), np.load(b)
    diff = [k for k in sorted(set(za.files) | set(zb.files)) if k not in za.files or k not in zb.files or za[k].shape != zb[k].shape or not np.array_equal(za[k], zb[k])]
    print(f, 'IDENTICAL' if not diff else 'DIFFERS in ' + ', '.join(diff))
PY
chown -R 1000:1000 $O
echo done > $O/DONE
