#!/usr/bin/env python3
"""启动 Pace、底盘、已知地图 AMCL 和 Nav2 全向导航."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """定位模式关闭 SLAM Toolbox，由 AMCL 独占 map 到 odom 的动态 TF."""
    package_share = FindPackageShare("kiwi_navigation")
    map_file = LaunchConfiguration("map")
    nav2_params = LaunchConfiguration("nav2_params_file")
    relay_params = LaunchConfiguration("relay_params_file")
    rviz_config = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_software_rendering = LaunchConfiguration("rviz_software_rendering")

    arguments = [
        DeclareLaunchArgument("map", description="已保存地图的绝对 YAML 路径"),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=PathJoinSubstitution([package_share, "config", "nav2.yaml"]),
        ),
        DeclareLaunchArgument(
            "relay_params_file",
            default_value=PathJoinSubstitution([package_share, "config", "cmd_relay.yaml"]),
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution([package_share, "config", "navigation.rviz"]),
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_software_rendering", default_value="true"),
    ]

    platform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("kiwi_slam"), "launch", "kiwi_slam.launch.py"])
        ),
        launch_arguments={
            "start_base": "true",
            "start_ekf": "true",
            "start_lidar": "true",
            "start_slam": "false",
            "start_monitor": "false",
            "start_description": "true",
            "use_rviz": "false",
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "localization_launch.py"]
            )
        ),
        launch_arguments={
            "map": map_file,
            "use_sim_time": "false",
            "autostart": "true",
            "params_file": nav2_params,
            "use_composition": "False",
            "use_respawn": "False",
            "log_level": "info",
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

    return LaunchDescription(arguments + [platform, localization, navigation, relay, rviz])
