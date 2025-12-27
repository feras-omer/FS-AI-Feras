# eufs_autonomy

C++ ROS 2 autonomy pipeline for the EUFS simulator:

**perception (LiDAR + camera) → mapping → centerline planning → pure pursuit control**

## What’s in this repo
This repository contains only *my* autonomy stack (nodes + launch).  
You will still need the EUFS simulator in a separate workspace (or the same workspace as a source dependency).

## Dependencies
- ROS 2 (tested on Ubuntu 20.04 + ROS 2 Galactic)
- PCL, OpenCV, Eigen3
- `cv_bridge`, `pcl_conversions`, `tf2_*`, `ackermann_msgs`

## Build (colcon)
Inside your ROS 2 workspace:

```bash
cd ~/your_ws/src
git clone <YOUR_GITHUB_REPO_URL> eufs_autonomy
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select eufs_autonomy
source install/setup.bash
```

## Run
```bash
ros2 launch eufs_autonomy pipeline.launch.py
```

## Notes on EUFS
This stack was developed and tested against the EUFS simulator and messages maintained by the Edinburgh University Formula Student team.
See their GitLab projects for setup instructions and licensing.
