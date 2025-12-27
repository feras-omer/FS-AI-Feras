from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="eufs_autonomy",
            executable="gps_localization_node",
            name="gps_localization",
            output="screen",
        ),
        Node(
            package="eufs_autonomy",
            executable="lidar_cone_detector",
            name="lidar_cone_detector",
            output="screen",
        ),
        Node(
            package="eufs_autonomy",
            executable="camera_cone_verifier",
            name="camera_cone_verifier",
            output="screen",
        ),
        Node(
            package="eufs_autonomy",
            executable="cone_mapper",
            name="cone_mapper",
            output="screen",
        ),
        Node(
            package="eufs_autonomy",
            executable="centerline_generator",
            name="centerline_generator",
            output="screen",
        ),
        Node(
            package="eufs_autonomy",
            executable="pure_pursuit_controller",
            name="pure_pursuit_controller",
            output="screen",
        ),
    ])
