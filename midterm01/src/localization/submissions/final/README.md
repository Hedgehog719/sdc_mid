# Final submission files (E3 zip)

| file | candidate | Kaggle public | produced by |
| --- | --- | --- | --- |
| result_1.csv | t1_online_v7_img.csv | 0.47 (57689088) | `tools/run_online.sh /root/data/Track1 T1` |
| result_2.csv | t2_online_v19_img.csv | 0.26 (57689091) | `tools/run_online.sh /root/data/Track2 T2` |
| result_3.csv | t3_online_vofill.csv | 0.22 (57688125) | `tools/run_online.sh /root/data/Track3 T3` |

Each file is byte-identical to the runner output in the current Docker image (`sdc_ros2_dev`, container recreated
2026-10-25), checked on 2026-10-27. The earlier T1 v7 (0.46) and T2 v19 (0.26) files came from the previous container
and are not bit-reproducible there any more (T1 first differs by 2.4e-5 m at frame 48 even with the v7 commit; the
chaotic T1 filter amplifies it). T3 adds the VO gap fill (tools/vo_fill.py) to the 0.22 soft-heading configuration;
same public score, changes mostly in the private part.

```text
1fd129f4a2e9eabf66978b958ef9ad4a7d16bf5954ab41c74c08cbf5602cf252  result_1.csv
c952716ce2837348b6170361c73d9616ca9a8b53e3d20c5df692ea0da39353f5  result_2.csv
7573d68d84820b77922e0454f072c9cf102bcbdd4670d96caa7c038906d6b155  result_3.csv
```

All three passed schema, consecutive-index, finite-value and wrapped-heading checks (3011 / 3299 / 3142 poses). All
methods are causal and online: frame k uses frames <= k, written once. Only public scores are known; the Kaggle
final selection is made by the user before 10/29 09:00 and is not changed automatically.
