#!/bin/bash
cd /root/midterm/src/localization
R="python3 tools/render_topdown.py"
$R /root/data/Track2 submissions/day7/track2/t2_pipe_v5.csv --b submissions/track2/0.17429__t2_opt10.csv --label_a 'pipeline v5' --label_b '0.17' --follow a --name td_pipe --frames 0:-1:3 --tag clean16 > work/mb/td_t2_pipe.log 2>&1
$R /root/data/Track2 submissions/day7/track2/t2_pipe_v5.csv --b submissions/track2/0.17429__t2_opt10.csv --label_a 'pipeline v5' --label_b '0.17' --follow b --name td_017 --frames 0:-1:3 --tag clean16 > work/mb/td_t2_017.log 2>&1
$R /root/data/Track1 submissions/day7/track1/t1_pipe_v5.csv --b submissions/track1/0.41123__t1_opt11.csv --label_a 'pipeline v5' --label_b '0.41' --follow a --name td_pipe --frames 0:-1:3 --tag clean15 > work/mb/td_t1_pipe.log 2>&1
$R /root/data/Track1 submissions/day7/track1/t1_pipe_v5.csv --b submissions/track1/0.41123__t1_opt11.csv --label_a 'pipeline v5' --label_b '0.41' --follow b --name td_041 --frames 0:-1:3 --tag clean15 > work/mb/td_t1_041.log 2>&1
chown -R 1000:1000 work/viewer work/video
echo render all done > work/mb/render_all.done
