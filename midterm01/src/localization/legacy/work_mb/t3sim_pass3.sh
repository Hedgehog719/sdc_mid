#!/bin/bash
# pass 3: fixes recomputed around the pass-2 trajectory (better prior for the accumulation), same fusion
cd /root/midterm/src/localization
for T in T3sim2 T3sim1; do
  cp work/t3/${T}_p2.csv work/t3/${T}p3_p1.csv
  for f in mk_S mk_D; do ln -sf ${T}_$f.npz work/t3/${T}p3_$f.npz; done
  bash work/mb/t3_pass2.sh /root/data/$T ${T}p3 work/t3/${T}p3_p1.csv 2>&1 | grep -v Warn | grep "start segment\|done\|kept"
done
python3 work/mb/t3_eval.py work/t3/T3sim2p3_p2.csv submissions/day8/track2/t2_pipe_v16.csv
python3 work/mb/t3_eval.py work/t3/T3sim1p3_p2.csv submissions/day8/track1/t1_pipe_v14m.csv
