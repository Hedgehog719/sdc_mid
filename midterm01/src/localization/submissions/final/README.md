# Final submission files (E3 zip)

| file | candidate | Kaggle public | produced by |
|---|---|---|---|
| result_1.csv | t1_online_v7.csv | 0.46 | `tools/run_online.sh /root/data/Track1 T1` |
| result_2.csv | t2_online_v19.csv | 0.26 | `tools/run_online.sh /root/data/Track2 T2` |
| result_3.csv | t3_online_tpg.csv | 0.24 | `tools/run_online.sh /root/data/Track3 T3` |

All three are byte-identical to the runner outputs of the committed code (causal, online: frame k uses frames <= k,
written once). The Kaggle selection must match (select these three before 10/29 09:00).
