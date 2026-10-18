# Next submission window (2026-10-19 08:00 Taiwan time) — user instruction

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
