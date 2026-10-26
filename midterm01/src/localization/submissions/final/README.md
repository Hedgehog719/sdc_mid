# Final submission files (E3 zip)

| file | candidate | Kaggle public | produced by |
| --- | --- | --- | --- |
| result_1.csv | t1_online_v7.csv | 0.46 | `tools/run_online.sh /root/data/Track1 T1` |
| result_2.csv | t2_online_v19.csv | 0.26 | `tools/run_online.sh /root/data/Track2 T2` |
| result_3.csv | t3_batch_soft_heading.csv | 0.22 (57675838) | `SDC_CAM_SYNC=1 SDC_CAM_SYNC_YAW=1 SDC_LOC_EXTRA="--yaw_soft_peak 0.2" bash tools/run_online.sh /root/data/Track3 T3` |

T3 is byte-identical to the verified Kaggle submission 57675838, with SHA-256
`ef5d8e693b3fb6618f5db217856c3e87e1de990fdfb6114cb267b51fa5d9821b`.

All three files passed schema, consecutive-index, finite-value and wrapped-heading checks, with 3011 / 3299 / 3142
poses respectively. Current SHA-256 values:

```text
6e9200189e3756f57df049db1d2a3ad72ea5ddfb6ea06a0a34dbcfd20ff78d49  result_1.csv
00b4a1c95b8e3c711a51f3634f392dd1ac478daa93d91383ee0bf291a4f98d68  result_2.csv
ef5d8e693b3fb6618f5db217856c3e87e1de990fdfb6114cb267b51fa5d9821b  result_3.csv
```

T3's full live heading bank and 600-frame causal-prefix check passed; configuration, source revision and core digest
are recorded in `../online/t3_batch_soft_heading.json`. All methods are causal and online: frame k uses frames <= k,
written once. T1/T2 remain the existing verified candidates, not unsubmitted local holdout experiments.

Only public scores are known; private accuracy is not guaranteed. Further candidates must not replace these files
without verification. Kaggle final selection must match these candidates before 10/29 09:00; this update does not
claim that the website's final selection has been changed automatically.

The overnight experiment run finished before 05:40 Asia/Taipei on 2026-10-27. No localization or Kaggle upload tasks
remain queued; no uploads are scheduled at or after the requested 08:00 cutoff. The final VO timing trial scored
0.31 and the T1 header trial tied 0.46, so neither replaced the simpler adopted candidates above.
