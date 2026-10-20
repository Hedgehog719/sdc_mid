#!/bin/bash
cd /root/midterm/src/localization
rm -rf work/viewer/Track2_dual_test work/video/Track2_dual_test.mp4 work/viewer/Track2_td_pipe work/video/Track2_td_pipe.mp4
R="python3 tools/render_topdown.py"
$R /root/data/Track2 submissions/day7/track2/t2_pipe_v5.csv --b submissions/track2/0.17429__t2_opt10.csv --label_a 'pipeline v5' --label_b '0.17' --layout dual --size 700 --name dual --frames 0:-1:3 --tag clean16 > work/mb/dual_t2.log 2>&1
$R /root/data/Track1 submissions/day7/track1/t1_pipe_v5.csv --b submissions/track1/0.41123__t1_opt11.csv --label_a 'pipeline v5' --label_b '0.41' --layout dual --size 700 --name dual --frames 0:-1:3 --tag clean15 > work/mb/dual_t1.log 2>&1
chown -R 1000:1000 work/viewer work/video
echo done > work/mb/render_dual.done
