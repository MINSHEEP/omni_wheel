#!/usr/bin/env python3
"""按活动底盘选择底盘/定位/导航链路和参数文件。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from kiwi_platform import load_profile, validate_profile
from kiwi_platform.launching import robot_actions, nav2_include, nav2_parameters, validate_map_file

PARAMS = {"base_params_file": "base", "lidar_params_file": "lidar", "nav2_params_file": "nav2"}


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _bool(context, name):
    value = _value(context, name)
    if value not in ("true", "false"):
        raise ValueError(f"{name} 必须是 true 或 false")
    return value == "true"


def _setup(context):
    map_file = validate_map_file(_value(context, "map"))
    selector = _value(context, "platform_file")
    overrides = {field: _value(context, argument) for argument, field in PARAMS.items()}
    selected = load_profile(selector, {"base": overrides["base"]}, required_keys=("base",))
    keys = ["base", "urdf", "lidar", "nav2"]
    if selected.odometry == "rf2o":
        keys.append("rf2o")
    profile = load_profile(selector, overrides, required_keys=tuple(keys))
    validate_profile(profile, "localization")
    actions = robot_actions(profile)
    actions += [
        nav2_include("localization_launch.py", params=profile.path("nav2"), map_file=map_file),
        nav2_include("navigation_launch.py", params=nav2_parameters(profile)),
    ]
    if _bool(context, "use_rviz"):
        share = Path(get_package_share_directory("kiwi_navigation"))
        config = _value(context, "rviz_config_file") or str(share / "config/navigation.rviz")
        actions.append(Node(package="rviz2", executable="rviz2", output="screen",
                            arguments=["-d", config], additional_env={
                                "QT_QPA_PLATFORM": "xcb",
                                "LIBGL_ALWAYS_SOFTWARE": "1" if _bool(context, "rviz_software_rendering") else "0",
                            }))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map", description="已保存地图的绝对 YAML 路径"),
        DeclareLaunchArgument("platform_file", default_value=""),
        *[DeclareLaunchArgument(field, default_value="") for field in PARAMS],
        DeclareLaunchArgument("rviz_config_file", default_value=""),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_software_rendering", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
