#!/bin/bash
# T3 start bank over heading offsets x forward offsets (validation): hypothesis runs of NH frames, then the full run of the
# MAP hypothesis at NH; combined by work/online/bank2_combine.py. usage: t3_bank2.sh <tag> "<heading offs>" "<fwd offs>"
cd /root/midterm/src/localization
TAG=$1; HYP=$2; FWD=$3
W=work/runs/$TAG; mkdir -p $W
export SDC_ROT_FIX=calib/cam_fix_v10a.json SDC_CAM_SYNC=1 SDC_CAM_SYNC_YAW=1
A=$(cat work/runs/T3_kvo97r/args.txt)
LOC="python3 tools/localize_online.py /root/data/Track3 work/runs/T3/mk_D.npz work/runs/T3/mk_S.npz"
VO=work/runs/T3/vo_fill.npz; NH=600; i=0
for o in $HYP; do for f in $FWD; do
  $LOC $W/hyp_${o}_$f.csv --vo $VO $A --init_offset=$o --init_fwd=$f --frames $NH --debug $W/hyp_${o}_${f}_dbg.csv > $W/hyp_${o}_$f.log 2>&1 &
  i=$((i + 1)); [ $((i % 3)) -eq 0 ] && wait
done; done
wait
SEL=$(python3 work/online/bank2_combine.py select $W $NH "$HYP" "$FWD")
set -- $SEL
$LOC $W/full.csv --vo $VO $A --init_offset=$1 --init_fwd=$2 --debug $W/debug.csv > $W/localize.log 2>&1
python3 work/online/bank2_combine.py combine $W $NH "$HYP" "$FWD" $1 $2 > $W/combine.log 2>&1
chown -R 1000:1000 $W
echo done > $W/DONE
