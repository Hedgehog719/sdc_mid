# Day 2 submissions (2026-10-18, resets 08:00 Taiwan time)

## Track1 (15 = 5 old + 5 mix + 5 decided after results)
Old candidates (most promising of yesterday's 16):
- old_bev_it2         2nd BEV iteration (prior = BEV-fused trajectory)
- old_bev_acc30       longer image accumulation (+-30 frames)
- old_bev_huber1.5    stricter outlier gating (best in drift simulation)
- old_bev_l05_f1.5    more forward weight
- old_bev_cf20        yaw complementary filter 20 s

"Mix" = all refinements: BEV re-matched around the best trajectory, rig yaw -1.5 deg (residual +0.5 deg
found in both tracks), suspension pitch model (pitch = -0.8 deg per m/s^2 of longitudinal accel, calibrated on
braking/accelerating frames), stricter gating.
- mix1_h15            lat 0.5 / fwd 2.0, Huber 1.5
- mix2_h2             same, Huber 2.0 (isolates the gating change)
- mix3_two_runs       two independent BEV runs fused (sigma x sqrt2)
- mix4_l04f15_cf20    lat 0.4 / fwd 1.5 + yaw filter 20 s
- mix5_q0.3           RTS process noise 0.3

Literature (non-holonomic constraint: no side slip -> small cross-track process noise):
- nhc_lon0.4_lat0.1   on top of mix1 (BEV pitch model, Huber 1.5)
- nhc_lon0.4_lat0.05
Remaining 3: combine winners; time-lag probes (candidates/track1/probe_lag+-0.1) if worth it.

## Track2 (14 + 1)
- par_cf15 / cf20 / cf45        yaw complementary filter
- par_q0.8 / q1.6 / qa0.5 / qa2 RTS process noise
- par_gs0.15                    GNSS measurement sigma
- bev_l0.7f3_h15 / l1.0f3_h15 / l1.5f4_h15   BEV (all refinements) with low weight + strict gating
- nhc_lon0.4_lat0.1 / nhc_lon0.8_lat0.1 / nhc_lon0.8_lat0.2   non-holonomic constraint
Remaining 1: combine winners.
(ZUPT/ZIHR tested: only the start of Track2 is stationary and RTS already holds it, change 2 mm -> dropped.)

## Offline findings behind the mix (Track2, 1 m / 2 deg noisy prior, median fix error)
| variant | median | sharp>0.02 median |
|---|---|---|
| 2D IPM, yaw -2.0 (yesterday) | 0.61 | 0.39 |
| 2D IPM, yaw -1.5 | 0.64 | 0.35 |
| + IMU dynamic pitch/roll (+) | 0.62 | 0.34 |
| 3D road-surface IPM (raster / quadratic) | 0.90-1.03 | 0.46-0.47 (rejected) |
Drift simulation (fusion params): Huber 1.5 best (0.816 vs 0.840 at 2.0).

## Suspension pitch model (Track2, 236 braking/accelerating frames |a|>0.4, 1 m / 2 deg noisy prior)
| k deg/(m/s^2) | +0.6 | +0.3 | 0 | -0.3 | -0.6 | -0.9 | -1.2 | -1.6 |
|---|---|---|---|---|---|---|---|---|
| NCC peak | 0.161 | 0.168 | 0.178 | 0.190 | 0.196 | 0.199 | 0.199 | 0.194 |
| median pos (all) | 1.01 | 0.74 | 0.69 | 0.63 | 0.61 | 0.67 | 0.64 | 0.79 |
| median pos (sharp>0.02) | 0.38 | 0.31 | 0.40 | 0.34 | 0.23 | 0.26 | 0.27 | 0.28 |
IMU high-pass pitch is uncorrelated with accel (r = -0.06): the AHRS attitude does not show nose dive.
Driving is smooth (|a| > 1 m/s^2 in 1-2 % of frames, max 1.2), so the effect is local.
