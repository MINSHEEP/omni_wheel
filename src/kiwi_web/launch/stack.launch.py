"""Owned robot stack. The web gate is the only /cmd_vel publisher."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from kiwi_web.config import PARAMETER_FILES


def setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in ("mapping", "navigation", "remote"):
        raise ValueError("Unsupported web stack mode")
    slam = Path(get_package_share_directory("kiwi_slam"))
    navigation = Path(get_package_share_directory("kiwi_navigation"))
    # Omit empty overrides: the owning package supplies its own defaults.
    overrides = {
        key: context.launch_configurations.get(key, "")
        for key in PARAMETER_FILES
        if key != "nav2_params_file"
    }
    overrides = {key: value for key, value in overrides.items() if value}
    actions = [
        GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(slam / "launch/kiwi_slam.launch.py")),
                    launch_arguments={
                        "use_rviz": "false",
                        "start_slam": str(mode == "mapping").lower(),
                        "start_monitor": str(mode != "remote").lower(),
                        "monitor_map_is_static": str(mode == "navigation").lower(),
                        **overrides,
                    }.items(),
                )
            ],
            forwarding=False,
        )
    ]
    if mode != "navigation":
        return actions
    arguments = {
        "map": LaunchConfiguration("map").perform(context),
        "raw_cmd_vel_topic": "/kiwi_web/nav_raw",
        "cmd_vel_topic": "/kiwi_web/nav_cmd_vel",
    }
    if context.launch_configurations.get("nav2_params_file"):
        arguments["nav2_params_file"] = context.launch_configurations["nav2_params_file"]
    actions.append(
        GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(navigation / "launch/navigation_core.launch.py")
                    ),
                    launch_arguments=arguments.items(),
                )
            ],
            forwarding=False,
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="remote"),
            DeclareLaunchArgument("map", default_value=""),
            *[DeclareLaunchArgument(key, default_value="") for key in PARAMETER_FILES],
            OpaqueFunction(function=setup),
        ]
    )
