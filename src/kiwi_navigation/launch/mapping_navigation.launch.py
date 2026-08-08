#!/usr/bin/env python3
"""启动可替换雷达、SLAM Toolbox、Kiwi 底盘和 Nav2 全向导航."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """让建图和导航只依赖统一 /scan，不直接绑定 E110 或 Mid-360 驱动."""
    navigation_share = FindPackageShare("kiwi_navigation")
    slam_share = FindPackageShare("kiwi_slam")
    lidar_type = LaunchConfiguration("lidar_type")
    nav2_params = LaunchConfiguration("nav2_params_file")
    relay_params = LaunchConfiguration("relay_params_file")
    slam_params = LaunchConfiguration("slam_params_file")
    monitor_params = LaunchConfiguration("monitor_params_file")
    base_params = LaunchConfiguration("base_params_file")
    ekf_params = LaunchConfiguration("ekf_params_file")
    rviz_config = LaunchConfiguration("rviz_config_file")
    start_base = LaunchConfiguration("start_base")
    start_ekf = LaunchConfiguration("start_ekf")
    start_lidar = LaunchConfiguration("start_lidar")
    start_slam = LaunchConfiguration("start_slam")
    start_monitor = LaunchConfiguration("start_monitor")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_software_rendering = LaunchConfiguration("rviz_software_rendering")

    chassis_ekf = PathJoinSubstitution(
        [FindPackageShare("kiwi_base"), "config", "ekf_chassis.yaml"]
    )
    mid360_ekf = PathJoinSubstitution([slam_share, "config", "ekf_mid360.yaml"])

    arguments = [
        DeclareLaunchArgument(
            "lidar_type",
            default_value="pace",
            description="雷达适配器：pace 或 mid360；算法层统一消费 /scan",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=PathJoinSubstitution([navigation_share, "config", "nav2.yaml"]),
            description="Nav2、DWB、costmap 和 AMCL 参数 YAML",
        ),
        DeclareLaunchArgument(
            "relay_params_file",
            default_value=PathJoinSubstitution(
                [navigation_share, "config", "cmd_relay.yaml"]
            ),
            description="Nav2 到底盘速度中继参数 YAML",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution([slam_share, "config", "slam_toolbox.yaml"]),
            description="SLAM Toolbox 参数 YAML",
        ),
        DeclareLaunchArgument(
            "monitor_params_file",
            default_value=PathJoinSubstitution([slam_share, "config", "monitor.yaml"]),
            description="SLAM 链路诊断参数 YAML",
        ),
        DeclareLaunchArgument(
            "base_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("kiwi_base"), "config", "kiwi_base.yaml"]
            ),
            description="底盘硬件和话题参数 YAML",
        ),
        DeclareLaunchArgument(
            "ekf_params_file",
            default_value=IfElseSubstitution(
                PythonExpression(["'", lidar_type, "' == 'mid360'"]),
                mid360_ekf,
                chassis_ekf,
            ),
            description="按雷达类型自动选择底盘或 Mid-360 IMU EKF YAML",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [navigation_share, "config", "navigation.rviz"]
            ),
            description="二维建图导航 RViz 配置",
        ),
        DeclareLaunchArgument("start_base", default_value="true"),
        DeclareLaunchArgument("start_ekf", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("start_monitor", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_software_rendering", default_value="true"),
    ]

    robot_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_share, "launch", "kiwi_robot.launch.py"])
        ),
        launch_arguments={
            "base_params_file": base_params,
            "ekf_params_file": ekf_params,
            "start_base": start_base,
            "start_ekf": start_ekf,
            "start_description": "true",
        }.items(),
    )

    radar_source = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([navigation_share, "launch", "radar_source.launch.py"])
        ),
        launch_arguments={
            "lidar_type": lidar_type,
            "start_lidar": start_lidar,
        }.items(),
    )

    slam = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        condition=IfCondition(start_slam),
        output="screen",
        parameters=[slam_params],
    )

    monitor = Node(
        package="kiwi_slam",
        executable="kiwi_slam_monitor",
        name="kiwi_slam_monitor",
        condition=IfCondition(start_monitor),
        output="screen",
        parameters=[monitor_params],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": "false",
            "autostart": "true",
            "params_file": nav2_params,
            # Nav2 Humble 官方 launch 用 PythonExpression 判断该值，必须是 False。
            "use_composition": "False",
            "use_respawn": "False",
            "log_level": "info",
        }.items(),
    )

    relay = Node(
        package="kiwi_navigation",
        executable="kiwi_nav_cmd_relay",
        name="kiwi_nav_cmd_relay",
        output="screen",
        parameters=[relay_params],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(use_rviz),
        output="screen",
        arguments=["-d", rviz_config],
        additional_env={
            "QT_QPA_PLATFORM": "xcb",
            "LIBGL_ALWAYS_SOFTWARE": IfElseSubstitution(
                rviz_software_rendering, "1", "0"
            ),
        },
    )

    return LaunchDescription(
        arguments + [robot_stack, radar_source, slam, monitor, navigation, relay, rviz]
    )
