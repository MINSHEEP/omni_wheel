"""Localization and navigation only; hardware and velocity arbitration are external.

All algorithm parameters and behavior trees belong to kiwi_navigation. Callers
choose velocity topics without copying or rewriting algorithm configuration.
"""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from kiwi_platform import load_profile, validate_profile
from kiwi_platform.launching import validate_map_file
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def setup(context):
    map_file = validate_map_file(LaunchConfiguration("map").perform(context))
    navigation = Path(get_package_share_directory("kiwi_navigation"))
    selector = LaunchConfiguration("platform_file").perform(context)
    overrides = {"nav2": LaunchConfiguration("nav2_params_file").perform(context),
                 "base": LaunchConfiguration("base_params_file").perform(context),
                 "lidar": LaunchConfiguration("lidar_params_file").perform(context)}
    selected = load_profile(selector, {"base": overrides["base"]}, required_keys=("base",))
    keys = ["base", "lidar", "nav2"]
    if selected.odometry == "rf2o":
        keys.append("rf2o")
    profile = load_profile(selector, overrides, required_keys=tuple(keys))
    validate_profile(profile, "localization")
    params = profile.path("nav2")
    raw_topic = LaunchConfiguration("raw_cmd_vel_topic").perform(context)
    output_topic = LaunchConfiguration("cmd_vel_topic").perform(context)
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    Path(get_package_share_directory("nav2_bringup"))
                    / "launch/localization_launch.py"
                )
            ),
            launch_arguments={
                "map": map_file,
                "params_file": params,
                "use_sim_time": "false",
                "autostart": "true",
                "use_composition": "False",
            }.items(),
        )
    ]
    rewritten = RewrittenYaml(
        source_file=params,
        root_key="",
        convert_types=True,
        param_rewrites={
            "default_nav_to_pose_bt_xml": str(
                navigation / "behavior_trees/navigate_to_pose_no_backup.xml"
            ),
            "default_nav_through_poses_bt_xml": str(
                navigation / "behavior_trees/navigate_through_poses_no_backup.xml"
            ),
        },
    )
    # Explicit remaps keep both the smoother output and recovery velocities
    # behind arbitration, without changing the existing RViz launch contracts.
    nodes = [
        ("nav2_controller", "controller_server", [("cmd_vel", raw_topic)]),
        ("nav2_smoother", "smoother_server", []),
        ("nav2_planner", "planner_server", []),
        ("nav2_behaviors", "behavior_server", [("cmd_vel", raw_topic)]),
        ("nav2_bt_navigator", "bt_navigator", []),
        (
            "nav2_velocity_smoother",
            "velocity_smoother",
            [("cmd_vel", raw_topic), ("cmd_vel_smoothed", output_topic)],
        ),
    ]
    for package, executable, remaps in nodes:
        actions.append(
            Node(
                package=package,
                executable=executable,
                name=executable,
                output="screen",
                parameters=[rewritten],
                remappings=remaps,
            )
        )
    actions.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "autostart": True,
                    "use_sim_time": False,
                    "node_names": [item[1] for item in nodes],
                }
            ],
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("platform_file", default_value=""),
            DeclareLaunchArgument("nav2_params_file", default_value=""),
            DeclareLaunchArgument("base_params_file", default_value=""),
            DeclareLaunchArgument("lidar_params_file", default_value=""),
            DeclareLaunchArgument("raw_cmd_vel_topic", default_value="/cmd_vel_nav"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            OpaqueFunction(function=setup),
        ]
    )
