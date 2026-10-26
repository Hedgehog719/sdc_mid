# Final submission files (E3 zip)

| file | candidate | Kaggle public | produced by |
|---|---|---|---|
| result_1.csv | t1_online_v7.csv | 0.46 | `tools/run_online.sh /root/data/Track1 T1` |
| result_2.csv | t2_online_v19.csv | 0.26 | `tools/run_online.sh /root/data/Track2 T2` |
| result_3.csv | t3_batch_soft_heading.csv | 0.22 (57675838) | `SDC_CAM_SYNC=1 SDC_CAM_SYNC_YAW=1 SDC_LOC_EXTRA="--yaw_soft_peak 0.2" bash tools/run_online.sh /root/data/Track3 T3` |

T3 is byte-identical to the verified Kaggle submission 57675838, with SHA-256
`ef5d8e693b3fb6618f5db217856c3e87e1de990fdfb6114cb267b51fa5d9821b`.
Its full live heading bank and 600-frame causal-prefix check passed; configuration, source revision and core digest
are recorded in `../online/t3_batch_soft_heading.json`. All methods are causal and online: frame k uses frames <= k,
written once. T1/T2 remain the existing verified candidates, not unsubmitted local holdout experiments.

Only public scores are known; private accuracy is not guaranteed. Further candidates must not replace these files
without verification. Kaggle final selection must match these candidates before 10/29 09:00; this update does not
claim that the website's final selection has been changed automatically.
