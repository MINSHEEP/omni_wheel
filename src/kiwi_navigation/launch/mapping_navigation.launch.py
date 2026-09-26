#!/usr/bin/env python3
"""活动底盘专属建图链路；已标定时同时启动该底盘的 Nav2。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from kiwi_platform import load_profile, validate_profile
from kiwi_platform.launching import nav2_include, nav2_parameters

PARAMS = {"base_params_file": "base", "lidar_params_file": "lidar", "slam_params_file": "slam",
          "mapping_gate_params_file": "gate", "monitor_params_file": "monitor", "nav2_params_file": "nav2"}


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _bool(context, name):
    value = _value(context, name)
    if value not in ("true", "false"):
        raise ValueError(f"{name} 必须是 true 或 false")
    return value == "true"


def _setup(context):
    selector = _value(context, "platform_file")
    overrides = {field: _value(context, argument) for argument, field in PARAMS.items()}
    selected = load_profile(selector, {"base": overrides["base"]}, required_keys=("base",))
    keys = ["base", "urdf", "lidar", "gate", "slam", "monitor"]
    if selected.odometry == "rf2o":
        keys.append("rf2o")
    profile = load_profile(selector, {key: value for key, value in overrides.items() if key in keys},
                           required_keys=tuple(keys))
    motion = validate_profile(profile, "mapping")
    if not _bool(context, "start_slam") or not _bool(context, "start_ekf"):
        raise ValueError("组合建图导航必须启动 SLAM Toolbox 和 EKF；已知图导航请用 localization_navigation.launch.py")
    slam_share = Path(get_package_share_directory("kiwi_slam"))
    navigation_share = Path(get_package_share_directory("kiwi_navigation"))
    slam_arguments = {
        "platform_file": str(profile.selector),
        "start_base": _value(context, "start_base"),
        "start_ekf": _value(context, "start_ekf"),
        "start_lidar": _value(context, "start_lidar"),
        "start_slam": _value(context, "start_slam"),
        "start_monitor": _value(context, "start_monitor"),
        "use_rviz": "false",
    }
    slam_arguments.update({argument: value for argument, field in PARAMS.items()
                           if field != "nav2" and (value := overrides[field])})
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(slam_share / "launch/kiwi_slam.launch.py")),
        launch_arguments=slam_arguments.items(),
    )]
    # 履带速度未标定时可以由遥控/原始电机命令手动建图，绝不启动 Nav2。
    if not motion and overrides["nav2"]:
        raise ValueError("履带未标定时 Nav2 不启动，nav2_params_file 不可使用")
    if profile.chassis == "kiwi" or motion:
        nav_profile = load_profile(selector, overrides, required_keys=tuple(keys + ["nav2"]))
        validate_profile(nav_profile, "localization")
        actions.append(nav2_include("navigation_launch.py", params=nav2_parameters(nav_profile)))
    if _bool(context, "use_rviz"):
        config = _value(context, "rviz_config_file") or str(navigation_share / "config/navigation.rviz")
        actions.append(Node(package="rviz2", executable="rviz2", output="screen",
                            arguments=["-d", config], additional_env={
                                "QT_QPA_PLATFORM": "xcb",
                                "LIBGL_ALWAYS_SOFTWARE": "1" if _bool(context, "rviz_software_rendering") else "0",
                            }))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("platform_file", default_value=""),
        *[DeclareLaunchArgument(field, default_value="") for field in PARAMS],
        DeclareLaunchArgument("rviz_config_file", default_value=""),
        *[DeclareLaunchArgument(field, default_value=value) for field, value in (
            ("start_base", "true"), ("start_ekf", "true"), ("start_lidar", "true"),
            ("start_slam", "true"), ("start_monitor", "false"),
            ("use_rviz", "true"), ("rviz_software_rendering", "false"),
        )],
        OpaqueFunction(function=_setup),
    ])
