#!/bin/bash
cd /root/midterm/src/localization
for T in T3sim2 T3sim1; do
  bash work/mb/t3_pass2.sh /root/data/$T $T
done
python3 work/mb/t3_eval.py work/t3/T3sim2_p2.csv submissions/day8/track2/t2_pipe_v16.csv
python3 work/mb/t3_eval.py work/t3/T3sim1_p2.csv submissions/day8/track1/t1_pipe_v14m.csv
