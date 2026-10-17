# SDC 2026 Midterm01 — Image-based Localization

Code for the NYCU SDC 2026 Midterm 1 localization competition (Track01 / Track02).
Data is not included (see `.gitignore`).

## Layout
```
Dockerfile, docker-compose.yml, ros_entrypoint.sh   ROS 2 Humble dev container (GPU)
midterm01/src/localization/                         colcon package (TA template, C++ NDT baseline)
  tools/                                            our pipeline (Python)
  run_pipeline.sh                                   reproduces result/result_{1,2}.csv
data/Track{1,2}/                                    datasets (mounted at /root/data)
```

## Run
```bash
docker compose up -d --build
docker exec -it sdc_midterm01 bash
bash /root/midterm/src/localization/run_pipeline.sh          # fast: sensor fusion only
FULL=1 bash /root/midterm/src/localization/run_pipeline.sh   # + map cache, YOLO masks, map matching
```
TA baseline (C++): `cd /root/midterm && colcon build && source install/setup.bash && ros2 launch localization localization.launch.py`

## Pipeline (tools/)
| file | purpose |
|---|---|
| `extract_bag.py` | dump `/localizer_node/gnss_pose` (4 Hz) and `/imu/data` (100 Hz) to CSV |
| `common.py` | GNSS/IMU interpolation, complementary-filter yaw, GNSS re-timing, tiled full-res map |
| `rts.py` | Kalman filter + RTS smoother, state [x, y, v], IMU heading as input, robust GNSS gating |
| `fuse.py` | final trajectory: GNSS + IMU yaw (complementary) + RTS (+ optional scan-matching terms) |
| `map_to_npy.py` | full-resolution map cache (ascii/binary PCD), no downsampling |
| `segment_vehicles.py` | YOLO11x-seg masks of road users on the 3 fisheye cameras |
| `camera_model.py` | FLU cloud -> fisheye pixel projection (validated with point colours) |
| `register.py` | per-frame GICP of 3-camera clouds to the full-res map with YOLO-mask removal, ego-motion temporal dynamic filter, map-consistency filter |
| `accum_register.py` | multi-frame accumulated submap registration |
| `gnss_variants.py`, `probe.py` | leaderboard probes (lag, yaw source, constant-offset error decomposition) |

## Findings (Kaggle public score, lower is better)
| step | Track1 | Track2 |
|---|---|---|
| TA baseline (NDT on raw_ply) | 1.791 | |
| GNSS interpolation | 1.311 | |
| + IMU yaw (median offset to GNSS) | 1.184 | 0.550 |
| + complementary yaw (low-passed GNSS-IMU offset) | 1.115 (10 s) | 0.462 (30 s, +RTS) |
| + Kalman/RTS smoothing with IMU heading | **1.087** (q 0.2) | 0.459 (q 0.4) |
| yaw reference = heading of travel | 1.100 | 0.350 |
| + yaw bias from offset probes (+0.29 / +0.22 deg) | 1.0786 | **0.3411** |
| + **map-to-image BEV road-marking fixes in RTS** (lat 0.5 m, fwd 2.0 m) | **1.0042** | 0.358 (weak: 0.342) |

### Map-to-image localization (scan matching against the LiDAR map)
The image-derived point clouds are too distorted for direct registration (offline test on Track2 with a
1.0 m / 2 deg perturbed prior: GICP makes it worse, 1.30 m -> 4.45 m; ground removal 4.07 m). Instead the map is
matched in the image domain:
1. `bev.py` builds a road-surface raster of the LiDAR map (cells on the local road plane, flat, so vegetation
   and kerbside clutter are rejected) and takes its reflectivity: lane lines, dashes, stop lines, crosswalks.
2. The three fisheye images are inverse-perspective-mapped onto the road plane (calibrated K/D/extrinsics,
   YOLO road users and the ego hood masked). A white top-hat filter extracts the painted markings.
3. `calib_bev.py` calibrates the rig mounting against a reference trajectory (camera height 1.5 m, yaw -2 deg).
4. `bev_match.py` accumulates image marking responses over +-20 frames using the smooth relative motion and
   finds the best yaw and x/y shift by normalised cross-correlation in the map raster.
5. The fixes enter the RTS smoother with anisotropic covariance and Huber gating, together with GNSS.

On Track2 with 1 m / 2 deg noise, the single-match median error drops from 1.23 m to 0.61 m. With simulated
Track1-like GNSS drift (0.95 m rms), fusion reaches 0.84 m. On the real Track1 the leaderboard score goes from
1.0786 to 1.0042. On Track2, where GNSS is already accurate (0.23 m), the fixes add noise, so it is not used there.

Error decomposition with constant-offset probes (`probe.py`, final settings before yaw bias):
| | pos RMSE | yaw RMSE | mean bias x / y (world) | fwd / lat (vehicle) | yaw |
|---|---|---|---|---|---|
| Track1 | 0.887 m | 1.00 deg | -0.04 / -0.02 m | +0.02 / -0.03 m | -0.29 deg |
| Track2 | 0.232 m | 0.59 deg | +0.04 / +0.04 m | | -0.22 deg |

Track1's remaining error is zero-mean, slowly wandering GNSS position error (no constant bias in world or vehicle
frame); only a reliable map match could remove it.

Negative results (kept for the report):
* GNSS time shift +-0.1 s: 1.78 / 1.73 -> stamps already aligned.
* Gaussian low-pass of GNSS x/y: cuts corners (sigma 3 s -> 2.39).
* Lever arm from init_guess - GNSS (-0.62, +0.21 m, identical on both tracks): worse (1.38 / 1.00) -> it is the injected noise.
* Yaw from the heading of the RTS path (T2 0.361) and GNSS re-timing (T2 0.342) did not beat 0.341.
* Image-cloud map matching (per frame or accumulated, with YOLO masks and dynamic filtering): corrections are not
  reliable (on Track2, where GNSS is accurate, they swing by +-5 m); as weak RTS measurements they give at most 0.001.

Observations: 3-camera MapAnything clouds are metric (camera height ~2.1 m), raw LoGeR clouds need scale ~3.5,
the map frame is tilted ~10 deg relative to the vehicle, GNSS fixes are on a regular 4 Hz grid with jittered stamps.
