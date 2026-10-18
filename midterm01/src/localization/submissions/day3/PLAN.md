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

## Road markings per camera + road region (kerbs / islands)
- Lane response computed inside each camera before blending (exposure / white-balance seams no longer create
  false edges): Track2 noisy-prior test median 0.64 -> 0.52 m, yaw 2.07 -> 1.73 deg.
- Road region: SegFormer road (+ vehicles; YOLO areas neutral) in BEV vs the map road surface bounded by kerbs
  (row scan of the detrended ground height). Lateral rms 0.55-0.58 m for the better half (prior 1.03 m).
- Track2 drift simulation: old lanes + curated poles 0.599 -> per-camera lanes 0.571 -> + road region 0.566.
- Track1 third iteration (per-camera lanes + poles round 2 + road) moves only 0.05-0.09 m from iter2: converged.

## Track1 start segment (frames 0-264, < 3 m/s)
Viewer review (frames 60/90/225): the map is clean there (no vehicles; the clutter came from the single-plane
ground model on a section whose road height varies 0.7 m within 30 m). Tested on the start segment (lane fixes,
frames 0-450, +-5 m search):
| variant | median fix | frame-to-frame jump | at search bound |
|---|---|---|---|
| frame window (current) | 1.07 m | 0.53 m | 11 % |
| **path-distance accumulation (+-10 m)** | **0.75 m** | **0.29 m** | **3 %** |
| + local ground | 1.40 m | 0.52 m | 18 % |
| + colour channels (yellow / red paint) | 0.72 m | 0.40 m | 13 % (Track2 also worse: 0.56 -> 0.89 m) |
User idea: the IMU heading hardly changes at the start, so hold yaw constant and keep the path straight there
(--start_hold 1.5: frames 0-264); it changes the start laterally by up to 0.3 m.

## Road surface from local slope (user idea) and the camera kerb side
Map road = smooth ground (local slope over 0.3 m minus the area's typical slope) connected to the vehicle; kerbs
cut it. Agreement with the image road segmentation: Track1 0.94 on clean sections (was 0.47-0.61), start 0.30-0.40
(was 0.06-0.14). Used for the road-region (kerb) feature only: as the road mask of lane matching it is worse
(Track2 noisy prior 0.56 -> 2.34 m: the connected region drops other-carriageway markings).
Camera side, 46-49 moving frames per track vs the map smooth road (median / worst quartile):
B2 0.684/0.456 (T1), 0.705/0.472 (T2); Mask2Former 0.669/0.482, 0.697/0.465; M2F - sidewalk 0.609/0.390;
Lab colour model 0.615/0.446; M2F + colour 0.685/0.527, 0.693/0.464 (best near range 0.70-0.73).
Agreement is flat with range (0.58-0.73), the rest is definitional (other carriageway behind a raised median,
flush sidewalks / driveways).
Drift simulation (0.95 m): lanes + poles 0.566; + kerb/road 0.559; without lanes (poles + kerb) 0.657; kerb only
0.875 -> lanes stay, kerbs added.

## Track1 (in order)
0. iter4_lane_pole_curb_hold   lanes (distance acc.) + curated poles + kerb/road (slope map) + start hold
0b. iter4_NOLANE_pole_curb_hold  user test: no lane markings (poles + kerb/road + start hold)
1. iter3_pcd_hold          distance-accumulated lanes + curated poles + start hold
2. iter3_pc                per-camera lanes + curated poles (round 2)
3. iter3_pc_hold           start hold only
4. iter3_pcd_lowspd        distance-accumulated lanes also used below 3 m/s
5. iter3_pc_road           + road region
6. curated_p50_f0.3        curated poles (fwd change vs best 0.52 m rms)
7. iter2_f0.3              second iteration
8. curated_occ_p50_f0.3    + occlusion check (occluders < 6 m, 2 m margin)
9-10. time-lag probes on the best of 1-8
then combinations.

## Track2
1-4. error decomposition probes on the current best (x+-, yaw+-) -> remaining pos vs yaw error and yaw bias
5. curated_p50_f0.5        (= yaw_q0.05_bev1.0_poles with BEV yaw 1.5)
6. yaw_q0.05_bev1.0_poles
7-8. yaw_q0.05_bev1.0, yaw_q0.07_bev1.0
then combinations.

## Road region caveat (viewer review, Track1 frames 60 / 225)
The image segmentation labels both carriageways as road while the kerb row scan stopped at the median, and
skipping tall blobs let the scan run into the scooter-lined arcade. A road-level definition (cells at the local
road height, detrended for the ~10 deg map tilt; raised = sidewalk / islands; tall = unknown) matches the
segmentation better on clean map sections (Track1 frame 800 corr 0.53) but near the Track1 start the map road is
covered by vehicles captured at mapping time, so there is nothing to match. Road region is therefore secondary.
