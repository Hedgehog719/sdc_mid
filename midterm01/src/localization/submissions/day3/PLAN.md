# Day 3 submissions (2026-10-19, resets 08:00 Taiwan time)

Current best: Track1 0.93699 (day2/combo/t1_final_q0.1_bev1.0), Track2 0.28263 (day2/track2/yaw_traj_q0.05_bev).

## New: pole landmarks (tools/poles.py + tools/pole_match.py)
Poles (street lights, sign / signal poles, utility poles, trunks) are extracted from the LiDAR map (vertical,
thin, isolated columns; T1 219, T2 256) and projected into the three fisheye cameras; the image edge energy across
each projected pole is scored over a forward/lateral grid, score grids are summed over +-30 frames (prior error is
slowly varying), YOLO masks remove pedestrians/vehicles. Pole azimuth hardly depends on camera pitch, so it
constrains the along-track position that lane markings cannot.

Track2 drift simulation (0.95 m GNSS drift):
| fixes | total | fwd | lat |
|---|---|---|---|
| GNSS only | 0.950 | | |
| + lane markings (current T1 method) | 0.837 | 0.726 | 0.417 |
| + poles | 0.699 | 0.483 | 0.505 |
| **+ lane markings + poles (top-50% peaks, fwd sigma 0.3)** | **0.577** | **0.472** | **0.332** |
Keeping weak pole matches (top 70%) can blow up (1.145) -> quality gating is essential.

## Hand-curated poles (pole_review_server.py, reviewed remotely)
Track1: 21 of 101 route poles rejected (building shapes, protruding signboards, unclear objects);
Track2: 9 of 129 rejected. Poles must also reach the ground now (drops hanging signboards), side cameras
yaw-corrected (+0.5 / -0.5 deg). Optional occlusion check: pole samples hidden behind nearer static map
structure (z-buffer of non-ground map points) are not scored; on Track2 (suburban, almost no occlusion, reference
only ~0.23 m accurate) it shows no difference, so it is tested on Track1 (urban) via the leaderboard.

## Track1 (in order)
1. curated_p50_f0.3        curated poles (fwd change vs best 0.52 m rms)
2. curated_occ_p50_f0.3    + occlusion check (occluders < 6 m, 2 m margin)
3. curated_p50_f0.5
4. pole_p50_f0.3           uncurated, for comparison
5-6. time-lag probes on the new best; then combinations.

## Track2
1. curated_p50_f0.5
2. pole_p50_f0.8
then combinations.
