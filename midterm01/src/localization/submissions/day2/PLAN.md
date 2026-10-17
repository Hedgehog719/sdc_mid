# Day 2 submissions (2026-10-18, resets 08:00 Taiwan time) — 12 per track first, 3 kept for combinations

Yaw is worth a lot: score = pos_rmse + 0.2 * yaw_rmse_deg. Probes give yaw rmse ~0.96 deg on Track1 (0.19 of
the score) and ~0.54 deg on Track2 (0.11, a third of the score), so yaw candidates get priority.

## Track1 (12 + 3)
| group | files | idea |
|---|---|---|
| BEV (old) | old_bev_it2, old_bev_acc30, old_bev_huber1.5 | 2nd iteration, longer accumulation, strict gating |
| BEV mix | mix1_h15, mix2_h2, mix3_two_runs, mix5_q0.3 | re-matched, rig yaw -1.5, suspension pitch model |
| NHC | nhc_lon0.4_lat0.1, nhc_lon0.4_lat0.05 | no side slip: small cross-track process noise |
| yaw | yaw_gnss_q0.05, yaw_traj_q0.05, yaw_traj_q0.05_bev | IMU yaw + RTS offset; speed-weighted motion heading (GNSS / BEV-fused trajectory) + BEV yaw fixes |

## Track2 (12 + 3)
| group | files |
|---|---|
| yaw | yaw_gnss_q0.02, yaw_gnss_q0.05, yaw_traj_q0.05, yaw_traj_q0.05_bev |
| params | par_cf15, par_cf45, par_q0.8, par_qa2, par_gs0.15 |
| NHC | nhc_lon0.8_lat0.1, nhc_lon0.8_lat0.2 |
| BEV | bev_l1.0f3_h15 |

Yaw candidates have their mean yaw aligned to the probed (bias-free) best submission (tools/align_yaw_mean.py).
Files not in these lists (mix4, par_cf20, par_q1.6, par_qa0.5, bev_l0.7/l1.5, old_bev_l05_f1.5, old_bev_cf20) are
kept as reserves.

## Offline findings
Suspension pitch model (Track2, 236 braking/accelerating frames, 1 m / 2 deg noisy prior):
| k deg/(m/s^2) | +0.6 | +0.3 | 0 | -0.3 | -0.6 | -0.9 | -1.2 | -1.6 |
|---|---|---|---|---|---|---|---|---|
| NCC peak | 0.161 | 0.168 | 0.178 | 0.190 | 0.196 | 0.199 | 0.199 | 0.194 |
| median pos (sharp>0.02) | 0.38 | 0.31 | 0.40 | 0.34 | 0.23 | 0.26 | 0.27 | 0.28 |
ZUPT/ZIHR: Track2 only stationary at the start, change 2 mm -> dropped. 3D road IPM rejected (0.90-1.03 vs 0.61).
