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

Leaderboard on 10-18 14:20: Track1 0.859 (team 13) vs ours 0.937; Track2 0.258 (team 6) vs ours 0.283.

## Second iteration (Track1)
BEV + pole matching re-run with prior = curated_p50_f0.3. It converges: pole fixes |fwd| median 0.89 -> 0.25 m,
BEV fwd 0.55 -> 0.42 m, lat 0.21 -> 0.17 m, so the first-round pole corrections look real.

## Track1 (in order)
1. curated_p50_f0.3        curated poles (fwd change vs best 0.52 m rms)
2. iter2_f0.3              second iteration (0.60 m vs best, 0.28 m vs round 1)
3. curated_occ_p50_f0.3    + occlusion check (occluders < 6 m, 2 m margin)
4. iter2_f0.5
5. curated_p50_f0.5
6. pole_p50_f0.3           uncurated, for comparison
7-8. time-lag probes on the best of 1-6
then combinations.

## Track2
1-4. error decomposition probes on the current best (x+-, yaw+-) -> remaining pos vs yaw error and yaw bias
5. curated_p50_f0.5        (= yaw_q0.05_bev1.0_poles with BEV yaw 1.5)
6. yaw_q0.05_bev1.0_poles
7-8. yaw_q0.05_bev1.0, yaw_q0.07_bev1.0
then combinations.
