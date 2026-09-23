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
from kiwi_platform import load_profile, validate_profile
from kiwi_platform.launching import validate_map_file


def setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in ("mapping", "navigation", "remote"):
        raise ValueError("Unsupported web stack mode")
    fields = {"base_params_file": "base", "lidar_params_file": "lidar",
              "slam_params_file": "slam", "mapping_gate_params_file": "gate",
              "monitor_params_file": "monitor", "nav2_params_file": "nav2"}
    profile = load_profile(LaunchConfiguration("platform_file").perform(context),
                           {field: context.launch_configurations.get(key, "")
                            for key, field in fields.items()})
    expected = LaunchConfiguration("expected_chassis").perform(context)
    if expected and expected != profile.chassis:
        raise ValueError("Web 服务启动后活动底盘已变化，请重启 Web 服务")
    validate_profile(profile, "mapping" if mode == "mapping" else "localization" if mode == "navigation" else "driver")
    map_file = validate_map_file(LaunchConfiguration("map").perform(context)) if mode == "navigation" else ""
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
                        "platform_file": str(profile.selector),
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
        "map": map_file,
        "platform_file": str(profile.selector),
        "raw_cmd_vel_topic": "/kiwi_web/nav_raw",
        "cmd_vel_topic": "/kiwi_web/nav_cmd_vel",
    }
    if context.launch_configurations.get("base_params_file"):
        arguments["base_params_file"] = context.launch_configurations["base_params_file"]
    if context.launch_configurations.get("lidar_params_file"):
        arguments["lidar_params_file"] = context.launch_configurations["lidar_params_file"]
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
            DeclareLaunchArgument("platform_file", default_value=""),
            DeclareLaunchArgument("expected_chassis", default_value=""),
            *[DeclareLaunchArgument(key, default_value="") for key in PARAMETER_FILES],
            OpaqueFunction(function=setup),
        ]
    )
