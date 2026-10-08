#!/usr/bin/env python3
"""履带 MID360 的二维建图导航或已知图 AMCL 导航。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from kiwi_platform import load_profile
from kiwi_platform.launching import nav2_include, validate_map_file
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _bool(context, name):
    value = _value(context, name)
    if value not in ("true", "false"):
        raise ValueError(f"{name} 必须是 true 或 false")
    return value == "true"


def _setup(context):
    mode = _value(context, "mode")
    if mode not in ("mapping", "localization"):
        raise ValueError("mode 必须是 mapping 或 localization")
    profile = load_profile(_value(context, "platform_file"), required_keys=("base",))
    if profile.chassis != "tracked_pwm":
        raise ValueError("MID360 导航目前仅支持 tracked_pwm 底盘")
    base = profile.data("base")["tracked_base_node"]["ros__parameters"]
    if _bool(context, "start_nav2") and not (
            base["hardware_calibrated"] is True and base["enable_motion"] is True
            and base["open_loop_test_enabled"] is False):
        raise ValueError("履带速度换算未完成标定或仍处于试车模式，不得启动 Nav2")

    slam_share = Path(get_package_share_directory("kiwi_slam"))
    nav_share = Path(get_package_share_directory("kiwi_navigation"))
    nav2_file = _value(context, "nav2_params_file") or str(nav_share / "config/mid360_tracked_nav2.yaml")
    slam_file = _value(context, "slam_params_file") or str(slam_share / "config/mid360_slam_toolbox.yaml")
    for label, path in (("Nav2", nav2_file), ("SLAM Toolbox", slam_file)):
        if not Path(path).is_file():
            raise ValueError(f"{label} 参数文件不存在：{path}")
    map_file = validate_map_file(_value(context, "map")) if mode == "localization" else None

    pipeline_args = {
        name: _value(context, name) for name in (
            "platform_file", "mount_file", "fastlio_params_file",
            "mode", "map_pcd", "start_base", "start_driver", "lidar_interface",
            "host_ip", "lidar_ip", "lidar_model",
            "scan_z_min", "scan_z_max",
        )
    }
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(slam_share / "launch/mid360_pipeline.launch.py")),
        launch_arguments=pipeline_args.items(),
    )]
    if mode == "mapping":
        actions.append(Node(package="slam_toolbox", executable="sync_slam_toolbox_node",
                            name="slam_toolbox", output="screen", parameters=[slam_file]))
    else:
        actions.append(nav2_include("localization_launch.py", params=nav2_file,
                                    map_file=map_file))

    if _bool(context, "start_nav2"):
        rewritten = RewrittenYaml(
            source_file=nav2_file, root_key="", convert_types=True,
            param_rewrites={
                "default_nav_to_pose_bt_xml": str(nav_share / "behavior_trees/navigate_to_pose_no_backup.xml"),
                "default_nav_through_poses_bt_xml": str(nav_share / "behavior_trees/navigate_through_poses_no_backup.xml"),
            },
        )
        actions.append(nav2_include("navigation_launch.py", params=rewritten))
    if _bool(context, "use_rviz"):
        actions.append(Node(package="rviz2", executable="rviz2", output="screen",
                            arguments=["-d", str(nav_share / "config/mid360_navigation.rviz")]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="mapping"),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("platform_file", default_value=""),
        DeclareLaunchArgument("mount_file", default_value=""),
        DeclareLaunchArgument("fastlio_params_file", default_value=""),
        DeclareLaunchArgument("nav2_params_file", default_value=""),
        DeclareLaunchArgument("slam_params_file", default_value=""),
        DeclareLaunchArgument("map_pcd", default_value="",
                              description="空值保存到工作区 maps/mid360_map.pcd"),
        DeclareLaunchArgument("start_base", default_value="true"),
        DeclareLaunchArgument("start_driver", default_value="true"),
        DeclareLaunchArgument("start_nav2", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("lidar_interface", default_value="enp2s0"),
        DeclareLaunchArgument("host_ip", default_value="192.168.1.50"),
        DeclareLaunchArgument("lidar_ip", default_value="192.168.1.117"),
        DeclareLaunchArgument("lidar_model", default_value="Mid360s"),
        DeclareLaunchArgument("scan_z_min", default_value="0.10"),
        DeclareLaunchArgument("scan_z_max", default_value="1.20"),
        OpaqueFunction(function=_setup),
    ])
