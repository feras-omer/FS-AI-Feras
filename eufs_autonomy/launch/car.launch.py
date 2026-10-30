"""Autonomy stack on the physical ADS-DV.

Publishes the sensor mounting transforms from config/ads_dv_car_tf.yaml and
starts the pipeline with the EKF state estimator and config/ads_dv_car.yaml.

Sensor drivers (LiDAR, ZED, GPS, IMU) and ros_can are started separately,
since they depend on the hardware fitted to the car.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def static_transforms(context):
    path = LaunchConfiguration("tf_config").perform(context)
    with open(path) as f:
        transforms = yaml.safe_load(f)["static_transforms"]

    nodes = []
    for t in transforms:
        x, y, z = (str(v) for v in t["xyz"])
        roll, pitch, yaw = (str(v) for v in t["rpy"])
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_" + t["child"],
            # Galactic argument order: x y z yaw pitch roll parent child
            arguments=[x, y, z, yaw, pitch, roll, t["parent"], t["child"]],
        ))
    return nodes


def generate_launch_description():
    share = get_package_share_directory("eufs_autonomy")
    return LaunchDescription([
        DeclareLaunchArgument(
            "tf_config",
            default_value=os.path.join(share, "config", "ads_dv_car_tf.yaml"),
            description="Sensor mounting transforms",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Open RViz with the autonomy layout",
        ),
        OpaqueFunction(function=static_transforms),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, "launch", "pipeline.launch.py")),
            launch_arguments={
                "vehicle": "ads_dv_car",
                "localization": "ekf",
                "rviz": LaunchConfiguration("rviz"),
            }.items(),
        ),
    ])
