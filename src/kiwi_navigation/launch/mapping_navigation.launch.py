#!/usr/bin/env python3
"""同时启动 Pace 二维建图、Kiwi 底盘和 Nav2 全向导航."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """组合现有二维建图入口和 Nav2，避免复制底盘、雷达与 SLAM 节点定义."""
    package_share = FindPackageShare("kiwi_navigation")
    nav2_params = LaunchConfiguration("nav2_params_file")
    relay_params = LaunchConfiguration("relay_params_file")
    rviz_config = LaunchConfiguration("rviz_config_file")
    start_base = LaunchConfiguration("start_base")
    start_ekf = LaunchConfiguration("start_ekf")
    start_lidar = LaunchConfiguration("start_lidar")
    start_slam = LaunchConfiguration("start_slam")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_software_rendering = LaunchConfiguration("rviz_software_rendering")

    arguments = [
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=PathJoinSubstitution([package_share, "config", "nav2.yaml"]),
            description="Nav2、DWB、costmap 和 AMCL 参数 YAML",
        ),
        DeclareLaunchArgument(
            "relay_params_file",
            default_value=PathJoinSubstitution([package_share, "config", "cmd_relay.yaml"]),
            description="Nav2 到底盘速度中继参数 YAML",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution([package_share, "config", "navigation.rviz"]),
            description="二维建图导航 RViz 配置",
        ),
        DeclareLaunchArgument("start_base", default_value="true"),
        DeclareLaunchArgument("start_ekf", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_software_rendering", default_value="true"),
    ]

    mapping_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("kiwi_slam"), "launch", "kiwi_slam.launch.py"])
        ),
        launch_arguments={
            "start_base": start_base,
            "start_ekf": start_ekf,
            "start_lidar": start_lidar,
            "start_slam": start_slam,
            "start_monitor": "false",
            "start_description": "true",
            "use_rviz": "false",
        }.items(),
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
            # Nav2 Humble 的官方 launch 用 PythonExpression 判断该值，必须使用
            # Python 可识别的 False，而不是小写 YAML 风格 false。
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

    return LaunchDescription(arguments + [mapping_stack, navigation, relay, rviz])
