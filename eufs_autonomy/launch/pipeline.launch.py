import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_nodes(context):
    vehicle = LaunchConfiguration("vehicle").perform(context)
    localization = LaunchConfiguration("localization").perform(context)

    params = os.path.join(
        get_package_share_directory("eufs_autonomy"), "config", vehicle + ".yaml")
    if not os.path.isfile(params):
        raise RuntimeError(f"No config for vehicle '{vehicle}': {params}")
    if localization not in ("ground_truth", "ekf"):
        raise RuntimeError(f"localization must be ground_truth or ekf, not '{localization}'")

    def node(package, executable, extra=None):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output="screen",
            parameters=[params] + ([extra] if extra else []),
        )

    nodes = [
        node("yolov5_cone_detector", "yolov5_cone_node",
             {"model_path": LaunchConfiguration("model_path")}),
        node("eufs_autonomy", "lidar_cone_detector"),
        node("eufs_autonomy", "camera_cone_verifier"),
        node("eufs_autonomy", "cone_mapper"),
        node("eufs_autonomy", "centerline_generator"),
        node("eufs_autonomy", "global_path_planner"),
        node("eufs_autonomy", "pure_pursuit_controller"),
    ]
    # ground_truth: the simulator publishes map -> base_footprint
    # (publish_gt_tf:=true). ekf: our estimator does, as on the real car.
    if localization == "ekf":
        nodes.append(node("eufs_autonomy", "state_estimator"))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "vehicle",
            default_value="eufs",
            description="Config in config/<vehicle>.yaml: eufs, ads_dv (sim) or ads_dv_car",
        ),
        DeclareLaunchArgument(
            "localization",
            default_value="ground_truth",
            description="ground_truth (sim publish_gt_tf:=true) or ekf (state_estimator)",
        ),
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
        OpaqueFunction(function=launch_nodes),
        Node(
            package="rviz2",
            executable="rviz2",
            name="autonomy_rviz",
            arguments=["-d", PathJoinSubstitution(
                [FindPackageShare("eufs_autonomy"), "config", "autonomy.rviz"])],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
