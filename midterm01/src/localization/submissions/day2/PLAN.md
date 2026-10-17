# Day 2 submissions (2026-10-18, resets 08:00 Taiwan time)

## Track1 (15 = 5 old + 5 mix + 5 decided after results)
Old candidates (most promising of yesterday's 16):
- old_bev_it2         2nd BEV iteration (prior = BEV-fused trajectory)
- old_bev_acc30       longer image accumulation (+-30 frames)
- old_bev_huber1.5    stricter outlier gating (best in drift simulation)
- old_bev_l05_f1.5    more forward weight
- old_bev_cf20        yaw complementary filter 20 s

"Mix" = all refinements: BEV re-matched around the best trajectory, rig yaw -1.5 deg (residual +0.5 deg
found in both tracks), IMU high-pass pitch/roll compensation (braking / bumps), stricter gating.
- mix1_h15            lat 0.5 / fwd 2.0, Huber 1.5
- mix2_h2             same, Huber 2.0 (isolates the gating change)
- mix3_two_runs       two independent BEV runs fused (sigma x sqrt2)
- mix4_l04f15_cf20    lat 0.4 / fwd 1.5 + yaw filter 20 s
- mix5_q0.3           RTS process noise 0.3

Remaining 5: combine winners; time-lag probes (candidates/track1/probe_lag+-0.1) if worth it.

## Track2 (8 + 7)
candidates/track2: cf15, cf20, cf45, q0.8, q1.6, qa0.5, qa2, gs0.15, then combinations.

## Offline findings behind the mix (Track2, 1 m / 2 deg noisy prior, median fix error)
| variant | median | sharp>0.02 median |
|---|---|---|
| 2D IPM, yaw -2.0 (yesterday) | 0.61 | 0.39 |
| 2D IPM, yaw -1.5 | 0.64 | 0.35 |
| + IMU dynamic pitch/roll (+) | 0.62 | 0.34 |
| 3D road-surface IPM (raster / quadratic) | 0.90-1.03 | 0.46-0.47 (rejected) |
Drift simulation (fusion params): Huber 1.5 best (0.816 vs 0.840 at 2.0).
