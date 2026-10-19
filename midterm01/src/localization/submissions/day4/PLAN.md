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

## 03:45 status / to do after the 27-min pause (user: full review of map + logic, then final runs + submit)
Leader: T1 0.760 (ours 0.93699), T2 0.257 (ours 0.28263). Quota resets 08:00 (both tracks used 15/15 today).
Done (sim T2-drift): 0.547 -> 0.326 (profile ground, min_speed 0, aniso k=0.01 lat, clean map, drop yaw-edge fixes).
Rejected: iteration with fused prior (0.345), map blob filter (0.335), isolated-point removal (no change),
pinned single height, per-block level correction (only pitch mode works -> map_clean6 for T1).
Pending results: c_bm_t2 (real T2, clean+profile), c_rds_drift / c6_rds_t1 (artefact-free slope kerb map).
T1 candidate ready: submissions/day4b/track1/new_c6e.csv (yaw mean aligned to the 0.93699 file).
Review checklist:
 1. map: clean_map on both maps (other stationary periods? T1 map end removed), 0.15 m double ground layer
    (lidar pitch bias) effects on lanes/kerb/pole heights, ego roof line, T2 start.
 2. bev_match: signs, sharp_f/sharp_l axes, edge handling, accumulation (PRIOR_REL) at low speed, dyn attitude.
 3. fuse: covariances, huber, gating, start_hold, yaw RTS inputs, yaw mean alignment, T1 time_shift (not applied).
 4. pole_match / road fixes with clean maps; kerb slope fix.
 5. T2: BEV position fixes on top of GNSS (consistency check of high-sharpness fixes first).
 6. decide finals, run, submit after 08:00 (estimated best first; stop a track once it improves).

## 04:05 review results (sim T2-drift 0.547 -> 0.262) and final candidates (submissions/day4/final/)
Review findings: yaw-edge BEV fixes (bug), process noise retuned (gnss 0.5 / qpos 0.1 / qlat 0.05; old
values were for weak fixes), heading input from a better-headed first pass (tight qlat follows the RTS
heading), D-pad artefact in the kerb slope map (bug), pole weights (pct 60, fwd 0.4). Lever arm checked on T2
(fixes vs unbiased GNSS: lat ~0, fwd +0.05..0.17): fine. Three T2 archive copies held same-named T1 files: fixed.
T2 start (stationary, 7.5 % of frames): clean-map BEV yaw (peak 0.35) and the map lane direction (-17.0 deg)
say the start yaw is -17.0..-17.4; current best has -18.6 -> stronger BEV yaw weight (0.7 deg).
Track1: 1) t1_fin.csv  2) t1_c6e_backup.csv (old process noise)  3) day4/track1/cal_planned.csv
Track2: 1) t2_pos.csv (GNSS + BEV/pole/kerb position fixes, gnss 0.3 / qpos 0.1 / qlat 0.05, heading from
  t2_yawbev0.7, BEV yaw 0.7 deg; small-drift sim 0.278 -> 0.244, never worse; start: fixes say 0.5 m right of
  GNSS at the stop next to a bus)  2) t2_yawbev0.7.csv (yaw only)  3) t2_yawonly.csv
Order rule: submit 1); if it beats the best -> stop that track and wait for the user; else submit 2), etc.

## 04:45 FINAL ORDER (supersedes the lists above)
Track1 start (frames 0-240): the map there was built from the mapping run's departure scans with a rotated
attitude; the BEV optimum is a stable -3.5 deg (search widened to +-6) and the user saw red map lines left at 30/60,
right + skewed at 180 (less at 240), normal after 840 - exactly what the matcher says. GT appears map-consistent
(explains T1's probed yaw RMSE 1.00 vs T2 0.59). Following the map there aligns the overlays (start_cmp).
Track1: 1) final/t1_fin_start.csv  2) final/t1_fin.csv  3) final/t1_c6e_backup.csv
Track2: 1) final/t2_pos2.csv (position+yaw fixes, start refined with +-6 deg search)  2) final/t2_yawbev0.7.csv
        3) final/t2_yawonly.csv
Rule: first file per track; if it beats the track's best (T1 0.93699, T2 0.28263) stop that track and wait for
the user; otherwise next file.

## 08:03 RESULTS
Track1: t1_fin_start 0.70845 (best was 0.93699) -> improved, stopped (1 submission used).
Track2: t2_pos2 0.32087, t2_yawbev0.7 0.32591, t2_pos2_bestyaw 0.29868 (best 0.28263) -> no improvement, stopped
  (3 used). On T2 (accurate GNSS) both the BEV position fixes (+0.016) and the stronger BEV yaw / map-consistent
  start yaw (+0.03..0.04) hurt; t2_yawonly not submitted (also changes yaw). Waiting for the user.

## Track2 diagnosis (user asked why T2 failed) -> t2_rest06 0.24752 (best was 0.28263)
Score ladder (best / t2_yawbev0.7 / t2_pos2_bestyaw) => with P~0.225 m, Y~0.29 deg: the start (frames 0-300,
stationary) position (0.41 m) and yaw (0.7-1.1 deg) changes did not help; the position change on the rest of the
track pointed against the existing error but overshot (optimal scale ~0.6).
Cause: the T2 frames 0-250 are recorded while stopped; matching them against map subsets built from the
stationary scans (early/mid/late) gives lateral +0.03..+0.15 m vs the GNSS solution while the departure scans
(the only ones left in map_clean) give -0.40 m; the mapping SLAM yaw drifted +1.0 -> -0.5 -> -1.5 deg during the
stop and jumped back after departure. T1 frames start after the stop (departure scans), so T1 is unaffected.
Fix: keep the GNSS solution while stationary (0-250, ramp to 330), apply 0.6 x the camera position correction
elsewhere, best yaw -> 0.24752 (predicted ~0.247).

## Probes (user: max 2)
T1 t1_fin 0.69910 (vs t1_fin_start 0.70845): the map-consistent start yaw cost ~0.009 -> new T1 best.
T2 t2_rest06 with start yaw -1.5 deg (late stationary scans) 0.28703 (vs 0.24752): the GNSS/IMU start yaw is right.
=> GT yaw is physical (IMU/GNSS-consistent); it does not follow the distorted map at the track starts.
Bests now: T1 0.69910 (t1_fin), T2 0.24752 (t2_rest06).

## Track1 segment probes prepared (waiting for the user's visual check before submitting)
Base F = t1_fin (0.69910); old B = 0.93699. Each probe replaces one segment of F by B (60-frame blends):
 h1_oldstart (0-300), h2_oldmid (300-840), h3_oldend (840-end) -> submissions/day5/track1/.
New-old: S1 fwd 0.58 (mean +0.35) lat 0.26 yaw 0.47; S2 fwd 0.74 (mean +0.65) lat 0.15 yaw 0.22;
S3 fwd 0.60 (mean +0.09) lat 0.31 yaw 0.37. Compare page: /compare/Track1 (a_old_0.937 vs b_new_0.699).
Results: h1_oldstart 0.61440 (new T1 best; the camera-corrected start was badly wrong, -0.085 for 10 % of the
frames), h2_oldmid 0.77510 (new pipeline much better on 300-840). h3 not submitted (user: new better after 540).
Conclusion: the map is fine where the mapping vehicle was moving; only the stop/departure region at the track
start is distorted -> there the GNSS/IMU solution must be used (same as Track2).

## Side-camera pole offsets (user spotted at T2 2700)
tools/pole_offsets.py on Track2 (660 confident poles): fl +10.9 px (~+1.0 deg), f +1.6 px, fr -8.7 px (~-0.8 deg).
A yaw-only fix (work/cam_rot_fix_pole.json: fl +0.47, f -0.36, fr -0.25) zeroes the pole offsets and makes the pole
fixes less biased vs the reference (fwd -0.27 -> -0.06 m) but lowers lane NCC ~8 % on the side cameras (both tracks)
and makes the T2-drift sim worse (0.262 -> 0.291; lanes alone 0.260). Shifting lane fixes back makes it worse too.
=> poles (upper image, at the side) and lanes (lower image) disagree: not a plain yaw error; likely side-camera
timing or fisheye distortion / roll about the optical axis. Calibration NOT changed (default cam_rot_fix.json).
Capture-time sync (SDC_CAM_SYNC=1; image timestamps: T2 fl -9.8 / fr +13.3 ms, T1 fl -1.9 / fr +12.3 ms):
T2-drift sim 0.262 -> 0.258-0.260 (small, consistent) -> adopt in the next full rerun.
Static pole-derived corrections after sync (fl/fr cameras 0.1-0.2 m further out, fr yaw -1 deg): sim 0.276-0.282,
lateral 0.150 -> 0.175 -> rejected; the extrinsic lateral positions are right for lanes, the residual pole
offsets (~5 px) come from elsewhere (pole map positions / fisheye distortion).

## Camera skew diagnosis (user request) - conclusion
Side cameras see poles at a constant angle offset (fl +0.70, fr -0.76 deg), identical on both tracks, independent
of speed (not timing / rolling shutter: pole tilt only 1-2 px), of range (not map pole positions) and of image
position (not fisheye distortion). Lanes reject the same rotation -> applied to the pole projection only
(pole_match --pole_cam_yaw fl=0.70,fr=-0.76): T2 pole fwd bias vs unbiased GNSS -0.22 -> -0.10 m, rms 0.67 -> 0.55,
peak 182 -> 190; T1 -0.46 -> -0.31. The apparent ~1.2 deg common rotation on Track1 is a position offset
(pole_match scoring rejects the rotation; probed yaw mean unbiased). Capture-time offsets (fl/fr up to 13 ms) are
real but small (sim 0.262 -> 0.258).
Candidates (only poles replaced, same recipes as the bests): submissions/day5/track1/t1_h1_pc.csv and
submissions/day5/track2/t2_rest06_pc.csv - both differ from the bests by ~0.04 m rms (expected effect ~0.002).
Pole-corrected: T1 t1_h1_pc 0.62128 (worse than 0.61440), T2 t2_rest06_pc 0.23453 (new best, was 0.24752).
Track1 option-2 probes: start boundary 420 -> 0.61902, 540 -> 0.64297 (300 stays best); correction ratio
0.7 -> 0.64485, 1.3 -> 0.66298 (1.0 = 0.61440; fitted optimum 0.96, gain ~0.0005).
Ladder decomposition of the T1 best: position RMSE 0.471 m (old solution 0.790 m), yaw part 0.143 (Y ~0.72 deg).
User (after these): stop submitting; only submit after careful offline confirmation and the user's OK.
Remaining today: T1 6, T2 9.

## Toward MVUE (user: approach the CRLB) - coloured GNSS noise
GNSS error vs our trajectories is almost entirely coloured: T1 0.58 m/axis, autocorr 0.86 @1 s, 0.36 @5 s, white part
~0.04 m; T2 0.14 m, autocorr 0.81 @1 s, 0.15 @5 s, white ~0.015 m. The white-noise model (sigma 0.5 at 4 Hz) over-trusts
GNSS averages ~3.5x. rts.py: optional first-order Gauss-Markov GNSS bias states (fuse --gnss_bias_tau/sigma/white).
The drift sim cannot show it (its reference is GNSS-based, ~0.2 m floor) -> tools/cv_fuse.py: block cross-validation
on real data (hold out every other block of map fixes, score held-out lane fixes).
T1 (blocks 40): lateral 0.174 -> 0.136, sharp-forward 0.224 -> 0.159 (tau 10 s, sigma 0.6); blocks 150: 0.355 -> 0.276,
0.564 -> 0.231. T2 (blocks 40): white with 0.6 shrink 0.144/0.141 -> bias model, no shrink 0.098/0.109; the 0.6
shrink compensated the wrong GNSS model (lateral wanted 1.0, forward 0 under the white model).
Candidates (positions only; start/stationary segments and yaw unchanged): submissions/day5/mvue/t1_gm.csv
(vs best fwd 0.10 lat 0.04 m), submissions/day5/mvue/t2_gm.csv (fwd 0.09 lat 0.07 m). NOT submitted (awaiting OK).
Bias-model candidates: T1 t1_gm 0.57658 (new best, was 0.61440); T2 t2_gm 0.25031 (worse than 0.23453). Remaining today: T1 5, T2 8.
