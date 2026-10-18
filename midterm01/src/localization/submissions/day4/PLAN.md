# Next submission window (2026-10-19 08:00 Taiwan time) — UPDATED 03:30 per user

User (03:25): "有更準確的 消除大問題的版本就取代原本要提交的檔案 ... 等預估最佳結果出現再交" ->
submit the estimated-best NEW version first (even after 08:00 if still computing); if a track improves, STOP
that track and wait for the user's reply. The cal_* files below are the fallback.

New pipeline (Track2 drift sim 0.547 -> 0.341):
- maps cleaned of the mapping vehicle's stationary scans (tools/clean_map.py; T1 18 %, T2 33 % of points:
  SLAM height/attitude drift + queued cars at the track starts) and the Track1 start levelled
  (tools/level_start.py --mode pitch, road level with frames 210-330 -> map_clean6)
- lane map with the road-profile ground mode (--ground_mode profile)
- BEV fixes at all speeds (--bev_min_speed 0) with per-fix anisotropic covariance from the NCC peak
  distinctiveness (--bev_aniso_k 0.01 --bev_aniso_lat)

## Original plan (fallback)

Submit FIRST the calibrated extended-BEV versions; if they beat the current best, STOP and wait for the user's reply.

| track | file | content |
|---|---|---|
| Track1 | day4/track1/cal_prevbest.csv | previous best config (0.93699) with calibrated extended-BEV lane fixes |
| Track1 | day4/track1/cal_planned.csv | planned method (lanes + curated poles + kerb + start hold), all fixes recomputed with calibration |
| Track2 | day4/track2/cal_prevbest.csv | previous best config (0.28263) with calibrated BEV yaw fixes |

Rule: after these, if a track's best improved -> no more submissions for that track until the user replies.
If not improved -> report and wait (day3/PLAN.md remains available).

## What changed (offline evidence)
- BEV extended to -5..30 m around the front camera (side cameras' near view used; valid pixels 46% -> 95%).
- Per-camera rotation calibration (work/cam_rot_fix.json): fl (-0.5, +0.5, +1.0), fr (+0.5, 0, -2.25),
  f (-0.5, +0.5, -0.5) deg (yaw, pitch, roll). Peak NCC +21-30% on Track2, +19-26% on Track1 (independent).
- Hand-marked pairs (41, /calib) solved jointly with per-frame pose nuisance give nearly the same rotations
  (fl (-0.5, +0.3, +0.7), f (-0.5, +0.3, +0.3), fr (+0.2, +0.2, -2.2)); camera height 1.52 m (not too high).
- Track2 drift simulation: 0.558 -> 0.547 (lateral 0.292 -> 0.254; lane fixes lateral rms 0.51 -> 0.28 m).
