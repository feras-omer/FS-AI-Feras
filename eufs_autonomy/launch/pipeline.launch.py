from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("yolov5_cone_detector"), "best.pt"]),
            description="Path to the YOLOv5 cone weights",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Open RViz with the autonomy layout (global path, cones)",
        ),
        Node(
            package="yolov5_cone_detector",
            executable="yolov5_cone_node",
            name="yolov5_cone_node",
            output="screen",
            parameters=[{"model_path": LaunchConfiguration("model_path")}],
        ),
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
            executable="global_path_planner",
            name="global_path_planner",
            output="screen",
        ),
        Node(
            package="eufs_autonomy",
            executable="pure_pursuit_controller",
            name="pure_pursuit_controller",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="autonomy_rviz",
            arguments=["-d", PathJoinSubstitution(
                [FindPackageShare("eufs_autonomy"), "config", "autonomy.rviz"])],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
