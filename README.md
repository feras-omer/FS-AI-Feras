# FS-AI-Feras

ROS 2 autonomy stack for the [EUFS simulator](https://gitlab.com/eufs/eufs_sim):

**perception (LiDAR + YOLOv5 camera) → mapping → centerline planning → pure pursuit control**

This repo contains only the autonomy stack. The simulator lives in its own workspace, and this repo is built as an overlay on top of it.

## How it works

```
EUFS sim (Gazebo)
 ├─ /velodyne_points ─► lidar_cone_detector ──► /lidar/cones            cone positions (velodyne frame)
 ├─ /zed/image_raw ───► yolov5_cone_node ─────► /zed/cones_detections   2D boxes + class
 │                      camera_cone_verifier ◄─┘  projects LiDAR cones into the image,
 │                                                 colours them from the YOLO box ─► /cones/verified
 ├─ TF map→base_footprint ─► cone_mapper ─────► /map/cones              persistent cone map (map frame)
 │                           centerline_generator ─► /map/centerline    local path ahead of the car (lap 1)
 │                           global_path_planner ──► /global_path       closed Bezier loop (after lap 1)
 └─ /cmd ◄─ pure_pursuit_controller                                     steering + speed, 20 Hz
```

Cone colour is carried in `pose.orientation.z` (`1` = blue = left, `2` = yellow = right).

**Lap 1 (explore):** the car follows a local centerline (blue/yellow midpoints ordered forward from the car) at up to `explore_speed`, while the mapper builds a persistent cone map. Cones seen 3+ times are kept for the whole run; colour is a majority vote.

**End of lap 1:** when the car is back within 4 m of its start after driving 40+ m, `global_path_planner` pairs every blue/yellow cone in the map, orders the midpoints around the loop, smooths them, and fits piecewise cubic Bezier curves (Catmull-Rom control points, so segments join smoothly). The result is published once on `/global_path` (`nav_msgs/Path`, transient local, so it stays available to RViz and late subscribers).

**Lap 2+ (race):** the controller follows the closed global path with a speed profile: `v = sqrt(max_lat_acc / curvature)`, limited by braking before corners and by acceleration after them. The planner logs every lap time.

On `small_track` this gives about 53 s for lap 1 and about 17 s for each race lap, with no cone hits.

| Package | Language | Contents |
|---|---|---|
| `eufs_autonomy/` | C++ | LiDAR detector, camera verifier, mapper, local centerline, global path planner, pure pursuit, GPS localization (not used by the controller), `pipeline.launch.py`, `config/autonomy.rviz` |
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
| Command mode | **velocity** | The controller sends speed only; in acceleration mode the car won't move |
| Use Simulated Perception | **unticked** | Ticking it removes the LiDAR and camera and publishes fake cones instead |
| Ground Truth TF | **ticked** | The mapper and controller need `map → base_footprint` |
| RViz / Gazebo GUI | optional | |

Or the equivalent single command:
```bash
ros2 launch eufs_launcher simulation.launch.py track:=small_track commandMode:=velocity \
  launch_group:=default publish_gt_tf:=true rviz:=true gazebo_gui:=true
```

### 2. Start the autonomy stack

```bash
ros2 launch eufs_autonomy pipeline.launch.py rviz:=true
# other weights: ... model_path:=/path/to/weights.pt
```
Wait for `YOLOv5 node ready`. `rviz:=true` opens a top-down RViz view with the global path (green), the lap-1 centerline (magenta), the cone map and the camera projection. To use the simulator's own RViz instead, add a **Path** display on `/global_path` and set its Durability Policy to **Transient Local**.

### 3. Arm the car

In the EUFS **Mission Control** rqt panel (opens with the simulator), select **TRACK_DRIVE** and click **Set Mission**. After about 5 s the state goes `AS_READY → AS_DRIVING` and the car starts driving.

Or from a terminal:
```bash
ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState "{ami_state: 14}"
```

Avoid **Manual Driving** and the **Robot Steering** panel: the first never arms, and the second also publishes to `/cmd`.

## Tuning

Parameters can be changed without rebuilding, e.g. `ros2 run eufs_autonomy pure_pursuit_controller --ros-args -p max_speed:=10.0`, or with `ros2 param set` while running.

| Node | Parameter | Default | Meaning |
|---|---|---|---|
| `pure_pursuit_controller` | `explore_speed` | 4.0 | Lap 1 speed cap (m/s) |
| | `max_speed` | 8.0 | Race speed cap (m/s) |
| | `max_lat_acc` | 5.0 | Cornering limit for the speed profile (m/s²) |
| | `max_brake` / `max_acc` | 4.0 / 2.5 | Braking / acceleration limits for the speed profile (m/s²) |
| | `base_lookahead`, `lookahead_speed_gain` | 2.5, 0.4 | Lookahead = base + gain × speed, clamped to [`min_lookahead`, `max_lookahead`] |
| `global_path_planner` | `min_lap_distance`, `lap_radius` | 40.0, 4.0 | Lap detection (m) |
| | `resample_spacing`, `smoothing_passes` | 2.0, 2 | Midpoint spacing and smoothing before the Bezier fit |
| | `sample_spacing` | 0.25 | Spacing of points on the published path (m) |
| both planners | `max_track_width`, `max_step`, `max_turn_deg` | 7.0, 6.0, 70 | Cone pairing and midpoint ordering limits |

## Debugging

- Stack logs: `Filtered points: N` (LiDAR), `Publishing N map cones` (mapper), `Centerline: left=N right=M` (planner), `Lap N complete: … m in … s` and `Global path published: …` (global planner), `switching to race mode` (controller).
- Sim state: `ros2 topic echo /ros_can/state_str`
- In RViz, `/debug/projection` shows each LiDAR cone projected into the camera image (blue/yellow, red = no colour).

| Symptom | Cause / fix |
|---|---|
| `environment variable 'EUFS_MASTER' does not exist` | `export EUFS_MASTER=$HOME/eufs_ws` |
| `package '…' not found` while launching Gazebo | A stale/broken install of that package in another sourced workspace; delete its `build/` and `install/` folders and open a fresh terminal |
| No `Filtered points` logs; `/velodyne_points` has no publisher | Simulated Perception is ticked (`launch_group:=no_perception`); untick it |
| State is `AS_DRIVING` but the car doesn't move | Command mode is acceleration, or no centerline (check `Centerline: left/right` counts) |
| `Could not close the loop` after lap 1 | Some cones were never coloured; the planner retries at the end of every lap |
| Long pip `InvalidMarker` traceback when YOLO starts | Harmless: YOLOv5's requirement auto-update fails on Python 3.8's pip; the model still loads |

## Notes on EUFS

Developed and tested against the EUFS simulator and messages maintained by the Edinburgh University Formula Student team. See their GitLab projects for setup instructions and licensing.

## License

MIT, see [LICENSE](LICENSE).
