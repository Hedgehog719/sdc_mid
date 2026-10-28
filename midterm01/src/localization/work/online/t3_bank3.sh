#!/bin/bash
# T3 heading-hypothesis bank with custom offsets (validation; mirrors the T3 branch of tools/run_online.sh with its
# arguments): usage t3_bank.sh <tag> "<offsets>" [workers] [extra localize_online options]
cd /root/midterm/src/localization
TAG=$1; HYP=$2; WK=${3:-3}; shift 3; X="$@"
W=work/runs/$TAG; mkdir -p $W
export SDC_ROT_FIX=calib/cam_fix_v10a.json SDC_CAM_SYNC=1 SDC_CAM_SYNC_YAW=1
A=$(cat ${T3ARGS:-work/runs/T3_kvo97r/args.txt})
LOC="python3 tools/localize_online.py /root/data/Track3 work/runs/T3/mk_D.npz work/runs/T3/mk_S.npz"
VO=work/runs/T3/vo_fill.npz; NH=600; PAIRS=""; i=0
for o in $HYP; do
  $LOC $W/hyp_$o.csv --vo $VO $A $X --init_offset=$o --frames $NH --debug $W/hyp_${o}_dbg.csv > $W/hyp_$o.log 2>&1 &
  PAIRS="$PAIRS $W/hyp_$o.csv:$W/hyp_${o}_dbg.csv"; i=$((i + 1))
  [ $((i % WK)) -eq 0 ] && wait
done
wait
OFFS=$(echo $HYP | tr ' ' ',')
BEST=$(python3 tools/hyp_select.py select $NH $OFFS $PAIRS 2> $W/hyp_select.log)
OB=$(echo $HYP | cut -d' ' -f$((BEST + 1)))
$LOC $W/full.csv --vo $VO $A $X --init_offset=$OB --debug $W/debug.csv > $W/localize.log 2>&1
python3 tools/hyp_select.py combine_mean $W/T3_mean.csv $NH $W/full.csv $OFFS $PAIRS 2>> $W/hyp_select.log
python3 tools/hyp_select.py combine $W/T3_map.csv $NH $W/full.csv $OFFS $PAIRS 2>> $W/hyp_select.log
echo "hypothesis $OB selected" >> $W/hyp_select.log
chown -R 1000:1000 $W
echo done > $W/DONE
