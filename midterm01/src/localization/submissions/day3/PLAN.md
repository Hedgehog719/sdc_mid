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

## Track1
1. pole_p50_f0.3   (moves along-track by 0.52 m rms vs best)
2. pole_p50_f0.5
3. pole_p60_f0.3
4-5. time-lag probes on the new best
then combinations.

## Track2
1. pole_p50_f0.5   (0.11 m fwd change; GNSS already accurate)
2. pole_p50_f0.8
then combinations.
