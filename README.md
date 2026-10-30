# FS-AI-Feras

ROS 2 autonomy stack for the [EUFS simulator](https://gitlab.com/eufs/eufs_sim) and the IMechE FS-AI **ADS-DV**:

**perception (LiDAR + YOLOv5 camera) → localisation (EKF) → mapping → planning → pure pursuit control**

This repo contains only the autonomy stack. The simulator lives in its own workspace, and this repo is built as an overlay on top of it. The same code runs on the EUFS car or the ADS-DV in simulation and on the physical ADS-DV; each vehicle is a config file (see [Vehicles](#vehicles)).

## How it works

```
EUFS sim or ADS-DV
 ├─ /imu, /gps, /ros_can/wheel_speeds ─► state_estimator ─► TF map→base_footprint   (EKF; or sim ground truth)
 ├─ /velodyne_points ─► lidar_cone_detector ──► /lidar/cones            cone positions (base_footprint)
 ├─ /zed/image_raw ───► yolov5_cone_node ─────► /zed/cones_detections   2D boxes + class
 │                      camera_cone_verifier ◄─┘  projects LiDAR cones into the image,
 │                                                 colours them from the YOLO box ─► /cones/verified
 ├─ TF map→base_footprint ─► cone_mapper ─────► /map/cones              persistent cone map (map frame)
 │                           centerline_generator ─► /map/centerline    local path ahead of the car (lap 1)
 │                           global_path_planner ──► /global_path       closed Bezier loop (after lap 1)
 │                                               ├─► /trajectory        same loop for the sim's RViz (grows during lap 1)
 │                                               └─► /laps              lap count
 ├─ /ros_can/state ─► pure_pursuit_controller ─► /cmd                   steering + speed + acceleration, 20 Hz
 └─ /ros_can/mission_completed, /ros_can/ebs ◄─┘
```

Cone colour is carried in `pose.orientation.z` (`1` = blue = left, `2` = yellow = right).

**Lap 1 (explore):** the car follows a local centerline (blue/yellow midpoints ordered forward from the car) at up to `explore_speed`, while the mapper builds a persistent cone map. Cones seen 3+ times are kept for the whole run; colour is a majority vote.

**End of lap 1:** when the car is back within 4 m of its start after driving 40+ m, `global_path_planner` pairs every blue/yellow cone in the map and orders the midpoints by where they lie along the route the car actually drove that lap (so the loop can't skip a section or dead-end where cones are missing). It then smooths them and fits piecewise cubic Bezier curves (Catmull-Rom control points, so segments join smoothly). The result is published on `/global_path` (`nav_msgs/Path`, transient local, republished every second) and, for the simulator's RViz, on `/trajectory`. During lap 1, `/trajectory` shows the path forming along the route driven so far.

**Lap 2+ (race):** the controller follows the closed global path with a speed profile: `v = sqrt(max_lat_acc / curvature)`, limited by braking before corners and by acceleration after them. The planner logs every lap time.

**Mission handling (`ros_can`, sim or ADS-DV):** the controller only drives in `AS_DRIVING`. It drives 10 laps for track drive and 1 for autocross (other missions are refused and the car holds), then brakes to a stop and publishes `/ros_can/mission_completed`. It commands both speed and an acceleration from a PI speed controller, so it works in velocity or acceleration command mode. If the car pose or the path goes stale it brakes, and after `ebs_timeout` it requests the EBS.

Results on `small_track` (simulation). The ADS-DV lap times are wall-clock times measured while the simulator ran at about 0.65× real time (two simulators sharing one laptop), so compare them with each other, not with the EUFS row. Lap times are now logged in simulation time.

| Car | Localisation | Lap 1 | Race laps | Mission |
|---|---|---|---|---|
| EUFS | ground truth | ~53 s | ~17 s | no cone hits |
| ADS-DV | ground truth | 76 s | 23.1–23.7 s | 10 laps, stopped, `AS_FINISHED`, no hits |
| ADS-DV | EKF (GPS + wheel speeds + steering) | 77.5 s | 21.8–22.5 s | 10 laps, stopped, `AS_FINISHED`, no hits; estimator error 0.28 m mean, < 1 m worst |
| ADS-DV | EKF, autocross | 73.7 s | n/a | 1 lap, stopped, `AS_FINISHED` |

| Package | Language | Contents |
|---|---|---|
| `eufs_autonomy/` | C++ | EKF state estimator, LiDAR detector, camera verifier, mapper, local centerline, global path planner, pure pursuit + mission handling, `pipeline.launch.py`, `car.launch.py`, vehicle configs, `config/autonomy.rviz`. (`gps_localization_node` is the original GPS-only node, kept for reference and not launched.) |
| `yolov5_cone_detector/` | Python | YOLOv5 node; weights are `best.pt` at the repo root (installed with the package) |

YOLO classes in `best.pt`: `0 blue_cone, 1 large_orange_cone, 2 orange_cone, 3 unknown_cone, 4 yellow_cone`.

## Requirements

- Ubuntu 20.04 + ROS 2 Galactic
- The EUFS simulator built in its own workspace (`eufs_sim`, `eufs_msgs`, `eufs_rviz_plugins`); see the [EUFS Getting Started Guide](https://gitlab.com/eufs/eufs_sim/-/wikis/Getting-Started-Guide). The examples below assume it is in `~/eufs_ws`.
- Python: PyTorch (CUDA optional, it falls back to CPU) and the YOLOv5 requirements:
  ```bash
  pip3 install torch torchvision
  pip3 install -r https://raw.githubusercontent.com/ultralytics/yolov5/master/requirements.txt
  ```
  The first run downloads the YOLOv5 code via `torch.hub` (internet needed once; it is cached in `~/.cache/torch/hub`).

## Build

```bash
git clone https://github.com/feras-omer/FS-AI-Feras ~/FS-AI-Feras
cd ~/FS-AI-Feras
source /opt/ros/galactic/setup.bash
source ~/eufs_ws/install/setup.bash          # simulator underlay
rosdep install --from-paths . --ignore-src -r -y
colcon build
```

The repo root is the colcon workspace (`build/`, `install/` and `log/` are gitignored).

> Don't also keep copies of `eufs_autonomy` / `yolov5_cone_detector` inside `~/eufs_ws/src`. Two installs of the same package compete, and stale ones break the Gazebo launch with `PackageNotFoundError`.

### Versions

`main` is always the latest working version. Each milestone is tagged, so you can go back to any of them:

| Tag | Contents |
|---|---|
| `pre-cleanup` | Original upload |
| `v0.1-cleanup` | Build fixes, YOLO started from the pipeline launch file |
| `v0.2-global-path` | Bezier global path built after lap 1, race laps on it |
| `v0.3-trajectory` | Path shown in the simulator's RViz, loop ordered along the driven route |
| `v0.4-ads-dv` | ADS-DV support: EKF localisation, mission handling (laps, stop, EBS), vehicle configs, `car.launch.py` |

```bash
git checkout v0.3-trajectory                 # or any tag; `git checkout main` to return
rm -rf build install log && colcon build     # clean rebuild, nodes differ between versions
```

## Run

Every terminal needs:
```bash
export EUFS_MASTER=$HOME/eufs_ws             # required by the EUFS track launch files
source /opt/ros/galactic/setup.bash
source ~/eufs_ws/install/setup.bash
source ~/FS-AI-Feras/install/setup.bash      # must be sourced last
```
(Use `setup.zsh` in zsh. Put these in your shell rc file to save typing.)

### 1. Start the simulator

Use either the launcher GUI:
```bash
ros2 launch eufs_launcher eufs_launcher.launch.py
```

| Setting | Value | Why |
|---|---|---|
| Command mode | velocity or **acceleration** | Both work; acceleration is closer to the real ADS-DV |
| Use Simulated Perception | **unticked** | Ticking it removes the LiDAR and camera and publishes fake cones instead |
| Ground Truth TF | **ticked** for `localization:=ground_truth`, **unticked** for `localization:=ekf` | The stack needs exactly one source of `map → base_footprint` |
| Robot | `eufs` or `ads-dv` | Match the `vehicle:=` config below |
| RViz / Gazebo GUI | optional | |

Or the equivalent single command:
```bash
ros2 launch eufs_launcher simulation.launch.py track:=small_track commandMode:=velocity \
  launch_group:=default publish_gt_tf:=true rviz:=true gazebo_gui:=true
```

### 2. Start the autonomy stack

```bash
ros2 launch eufs_autonomy pipeline.launch.py rviz:=true                              # EUFS car, ground truth
ros2 launch eufs_autonomy pipeline.launch.py vehicle:=ads_dv localization:=ekf       # ADS-DV, own localisation
# other weights: ... model_path:=/path/to/weights.pt
```
Wait for `YOLOv5 node ready`. `rviz:=true` opens a top-down RViz view with the global path (green), the lap-1 centerline (magenta), the cone map and the camera projection.

To see the path in the **simulator's RViz** instead, tick the **WaypointArrayStamped** display (it is already in the EUFS layout, just disabled). It shows the path growing during lap 1 and the closed loop afterwards. To keep that setting, use *File → Save Config As* `~/.rviz2/default.rviz`; the simulator loads that file instead of its default layout when it exists.

### 3. Arm the car

In the EUFS **Mission Control** rqt panel (opens with the simulator), select **TRACK_DRIVE** and click **Set Mission**. After about 5 s the state goes `AS_READY → AS_DRIVING` and the car starts driving.

Or from a terminal:
```bash
ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState "{ami_state: 14}"
```

Avoid **Manual Driving** and the **Robot Steering** panel: the first never arms, and the second also publishes to `/cmd`.

## Vehicles

Each vehicle is one parameter file in `eufs_autonomy/config/`, loaded by every node:

| `vehicle:=` | For | Notes |
|---|---|---|
| `eufs` (default) | EUFS car in the simulator | `robot_name:=eufs` |
| `ads_dv` | ADS-DV in the simulator | `robot_name:=ads-dv`; sensor positions from the ADS-DV sensor sheet |
| `ads_dv_car` | the physical ADS-DV | started by `car.launch.py`; lines marked `CHECK` must be verified on the car |

`localization:=ground_truth` uses the simulator's perfect pose (`publish_gt_tf:=true`). `localization:=ekf` runs `state_estimator`, which is what the real car uses:

- **Inputs:** rear wheel speeds and steering from `/ros_can/wheel_speeds`, yaw rate from the IMU (or from steering in the simulator, whose IMU under-reads yaw rate by ~40%), and GPS.
- **Model:** kinematic bicycle at `base_footprint` (sideslip in corners from the steering angle), with a learned yaw-rate scale (absorbs IMU scale error or understeer) and gyro bias.
- **Frames:** `map` is the car's start pose, so no compass is needed. The start GPS position is averaged while the car is stationary; the rotation between `map` and east/north is found after the first 5 m and then refined. The GPS antenna offset is read from TF.
- **Robustness:** GPS jumps are rejected (5σ gate) unless they persist; the pose is frozen while stationary.

### Physical ADS-DV bring-up

1. Start the sensor drivers (LiDAR, ZED, GPS, IMU) and EUFS `ros_can` for the ADS-DV. These depend on the hardware fitted, so they are not launched here.
2. Edit `config/ads_dv_car.yaml` and `config/ads_dv_car_tf.yaml`: every line marked `CHECK` (topic names, frame ids, measured sensor positions, tyre radius). `ros2 topic list` and `ros2 run tf2_tools view_frames` show what the drivers publish.
3. Rebuild, then launch:
   ```bash
   ros2 launch eufs_autonomy car.launch.py rviz:=true
   ```
   This publishes the sensor mounting transforms and starts the stack with the EKF. Until `ros_can` reports `AS_DRIVING`, it commands speed 0 and full brake.
4. Checks before driving, wheels off the ground or in a safe area:
   - `ros2 run tf2_ros tf2_echo map base_footprint` updates as the car is pushed, and the heading turns the right way.
   - `/debug/projection` shows LiDAR cones landing on the cones in the image (if not, fix the camera transform or `optical_frame`).
   - `Filtered points` is small with no cones around (if the ground gets through, raise `min_height`).
5. First runs: autocross (1 lap) at the conservative limits in `ads_dv_car.yaml`, then raise `explore_speed`, `max_speed` and `max_lat_acc` gradually.

## Tuning

Parameters can be changed without rebuilding, e.g. `ros2 run eufs_autonomy pure_pursuit_controller --ros-args -p max_speed:=10.0`, or with `ros2 param set` while running.

| Node | Parameter | Default | Meaning |
|---|---|---|---|
| `state_estimator` | `gps_noise` | 0.3 | GPS noise per axis (m); ~0.02 for RTK |
| | `yaw_rate_source` | imu | `imu`, or `steering` (v·tan δ / wheelbase) |
| | `gyro_noise`, `speed_noise` | 0.05, 0.2 | Process noise of yaw rate and speed |
| | `steering_delay` | 0.0 | Delay from steering to the car turning (s); 0.085 in the sim |
| | `ref_from_rear_axle` | 0.765 | Distance of `base_footprint` ahead of the rear axle (m) |
| `pure_pursuit_controller` | `explore_speed` | 4.0 | Lap 1 speed cap (m/s) |
| | `max_speed` | 8.0 | Race speed cap (m/s) |
| | `max_lat_acc` | 5.0 | Cornering limit for the speed profile (m/s²) |
| | `max_brake` / `max_acc` | 4.0 / 2.5 | Braking / acceleration limits for the speed profile (m/s²) |
| | `base_lookahead`, `lookahead_speed_gain` | 2.5, 0.4 | Lookahead = base + gain × speed, clamped to [`min_lookahead`, `max_lookahead`] |
| | `speed_kp`, `speed_ki` | 1.5, 0.3 | PI speed controller → acceleration command |
| | `max_acc_cmd`, `max_brake_cmd` | 3.0, 5.0 | Acceleration command limits (m/s²) |
| | `laps_track_drive`, `laps_autocross` | 10, 1 | Laps before stopping |
| | `pose_timeout`, `path_timeout`, `ebs_timeout` | 0.5, 1.0, 2.0 | Brake when inputs are this stale (s); request EBS after `ebs_timeout` |
| | `require_driving_state` | true | Only drive in `AS_DRIVING` (set false to test without `ros_can`) |
| `lidar_cone_detector` | `min_height`, `max_height` | 0.03, 0.5 | Cone points' height above the ground (m), in `base_footprint` |
| | `min_range`, `max_range`, `max_cone_width` | 1.0, 12.0, 0.5 | Range window and largest cluster accepted as a cone (m) |
| `global_path_planner` | `min_lap_distance`, `lap_radius` | 40.0, 4.0 | Lap detection (m) |
| | `max_offset` | 3.5 | Midpoints further than this from the driven route are ignored (m) |
| | `max_gap` | 12.0 | Largest allowed gap when closing the loop across the start line (m) |
| | `resample_spacing`, `smoothing_passes` | 2.0, 2 | Midpoint spacing and smoothing before the Bezier fit |
| | `sample_spacing` | 0.25 | Spacing of points on the published path (m) |
| both planners | `max_track_width` | 7.0 | Largest blue-yellow distance paired into a midpoint (m) |
| `centerline_generator` | `max_step`, `max_gap`, `max_turn_deg` | 6.0, 12.0, 70 | Lap-1 ordering: normal step, gap-bridging step (orange start line, cones beside the car), max turn per step |

## Debugging

- Stack logs: `Filtered points: N` (LiDAR), `Publishing N map cones` (mapper), `Centerline: left=N right=M` (planner), `Lap N complete: … m in … s` and `Global path published: …` (global planner), `switching to race mode` (controller).
- Sim state: `ros2 topic echo /ros_can/state_str`
- In RViz, `/debug/projection` shows each LiDAR cone projected into the camera image (blue/yellow, red = no colour).

| Symptom | Cause / fix |
|---|---|
| `environment variable 'EUFS_MASTER' does not exist` | `export EUFS_MASTER=$HOME/eufs_ws` |
| `package '…' not found` while launching Gazebo | A stale/broken install of that package in another sourced workspace; delete its `build/` and `install/` folders and open a fresh terminal |
| No `Filtered points` logs; `/velodyne_points` has no publisher | Simulated Perception is ticked (`launch_group:=no_perception`); untick it |
| State is `AS_DRIVING` but the car doesn't move | No centerline yet (check `Centerline: left/right` counts), or the mission isn't track drive/autocross |
| `Braking: pose … old, path … old`, then `requesting EBS` | The car pose (TF) or path stopped updating; check `state_estimator` / ground-truth TF and perception |
| Car drives off the map with `localization:=ekf` | Ground Truth TF is also ticked (two sources of `map → base_footprint`); untick it |
| `Rejected GPS fix` repeatedly | GPS multipath or a bad fix; if it persists the estimator accepts fixes again |
| `Could not build the loop` after lap 1 | Some cones were never coloured; the planner retries with the next lap's route |
| Path missing in the simulator's RViz | Tick the **WaypointArrayStamped** display (topic `/trajectory`) |
| Long pip `InvalidMarker` traceback when YOLO starts | Harmless: YOLOv5's requirement auto-update fails on Python 3.8's pip; the model still loads |

## Notes on EUFS

Developed and tested against the EUFS simulator and messages maintained by the Edinburgh University Formula Student team. See their GitLab projects for setup instructions and licensing.

## License

MIT, see [LICENSE](LICENSE).
