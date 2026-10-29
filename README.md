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
 ├─ TF map→base_footprint ─► cone_mapper ─────► /map/cones              cones in the map frame
 │                           centerline_generator ─► /map/centerline    midpoints of left/right cones
 ├─ /gps ─► gps_localization_node ─► /gps/odom
 └─ /cmd ◄─ pure_pursuit_controller                                     steering + speed, 20 Hz
```

Cone colour is carried in `pose.orientation.z` (`1` = blue, `2` = yellow).

| Package | Language | Contents |
|---|---|---|
| `eufs_autonomy/` | C++ | LiDAR detector, camera verifier, mapper, centerline, pure pursuit, GPS localization, `pipeline.launch.py` |
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
ros2 launch eufs_autonomy pipeline.launch.py
# other weights: ros2 launch eufs_autonomy pipeline.launch.py model_path:=/path/to/weights.pt
```
Wait for `YOLOv5 node ready`.

### 3. Arm the car

In the EUFS **Mission Control** rqt panel (opens with the simulator), select **TRACK_DRIVE** and click **Set Mission**. After about 5 s the state goes `AS_READY → AS_DRIVING` and the car starts driving.

Or from a terminal:
```bash
ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState "{ami_state: 14}"
```

Avoid **Manual Driving** and the **Robot Steering** panel: the first never arms, and the second also publishes to `/cmd`.

## Debugging

- Stack logs: `Filtered points: N` (LiDAR), `Publishing N map cones` (mapper), `Centerline: left=N right=M` (planner).
- Sim state: `ros2 topic echo /ros_can/state_str`
- In RViz, add `/debug/projection` (Image: projected cones; blue/yellow/red = unknown), `/map/cones` and `/map/centerline` (PoseArray).

| Symptom | Cause / fix |
|---|---|
| `environment variable 'EUFS_MASTER' does not exist` | `export EUFS_MASTER=$HOME/eufs_ws` |
| `package '…' not found` while launching Gazebo | A stale/broken install of that package in another sourced workspace; delete its `build/` and `install/` folders and open a fresh terminal |
| No `Filtered points` logs; `/velodyne_points` has no publisher | Simulated Perception is ticked (`launch_group:=no_perception`); untick it |
| State is `AS_DRIVING` but the car doesn't move | Command mode is acceleration, or no centerline (check `Centerline: left/right` counts) |
| Long pip `InvalidMarker` traceback when YOLO starts | Harmless: YOLOv5's requirement auto-update fails on Python 3.8's pip; the model still loads |

## Notes on EUFS

Developed and tested against the EUFS simulator and messages maintained by the Edinburgh University Formula Student team. See their GitLab projects for setup instructions and licensing.

## License

MIT, see [LICENSE](LICENSE).
