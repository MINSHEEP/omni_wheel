#!/usr/bin/env python3
"""旧命令兼容层；实际建图和导航分别进入独立 launch，选择来自活动 YAML。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from kiwi_platform import load_profile


def _setup(context):
    profile = load_profile(LaunchConfiguration("platform_file").perform(context))
    selected = LaunchConfiguration("chassis").perform(context)
    if selected and selected != profile.chassis:
        raise ValueError("旧 chassis 参数与 active_platform.yaml 冲突；请修改活动选择文件")
    mode = LaunchConfiguration("mode").perform(context)
    targets = {
        "driver": ("kiwi_slam", "kiwi_robot.launch.py"),
        "mapping": ("kiwi_navigation", "mapping_navigation.launch.py"),
        "localization": ("kiwi_navigation", "localization_navigation.launch.py"),
    }
    if mode not in targets:
        raise ValueError("mode 必须是 driver、mapping 或 localization")
    package, name = targets[mode]
    args = {"platform_file": str(profile.selector)}
    if mode == "driver":
        args.update(start_ekf="false", start_lidar="false")
    if mode == "localization":
        args["map"] = LaunchConfiguration("map").perform(context)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory(package)) / "launch" / name)),
        launch_arguments=args.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("platform_file", default_value=""),
        DeclareLaunchArgument("chassis", default_value=""),
        DeclareLaunchArgument("mode", default_value="mapping"),
        DeclareLaunchArgument("map", default_value=""),
        OpaqueFunction(function=_setup),
    ])
