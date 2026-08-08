#!/usr/bin/env python3
"""根据雷达型号启动驱动，并输出统一的 `/scan` 和可选 IMU 接口."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """只负责雷达适配，SLAM、Nav2 和底盘不依赖具体驱动包."""
    kiwi_slam_share = FindPackageShare("kiwi_slam")
    kiwi_navigation_share = FindPackageShare("kiwi_navigation")
    lidar_type = LaunchConfiguration("lidar_type")
    start_lidar = LaunchConfiguration("start_lidar")

    pace_params = LaunchConfiguration("pace_params_file")
    mid360_params = LaunchConfiguration("mid360_params_file")
    mid360_network = LaunchConfiguration("mid360_network_file")
    mid360_scan_params = LaunchConfiguration("mid360_scan_params_file")
    mid360_imu_params = LaunchConfiguration("mid360_imu_params_file")

    arguments = [
        DeclareLaunchArgument(
            "lidar_type",
            default_value="pace",
            description="雷达适配器：pace 或 mid360",
        ),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument(
            "pace_params_file",
            default_value=PathJoinSubstitution([kiwi_slam_share, "config", "lds_e110.yaml"]),
            description="E110 驱动参数",
        ),
        DeclareLaunchArgument(
            "mid360_params_file",
            default_value=PathJoinSubstitution(
                [kiwi_slam_share, "config", "livox_mid360s.yaml"]
            ),
            description="Mid-360 Livox 驱动参数",
        ),
        DeclareLaunchArgument(
            "mid360_network_file",
            default_value=PathJoinSubstitution(
                [kiwi_slam_share, "config", "mid360s_network.json"]
            ),
            description="Mid-360 网络 JSON",
        ),
        DeclareLaunchArgument(
            "mid360_scan_params_file",
            default_value=PathJoinSubstitution(
                [kiwi_navigation_share, "config", "radar_mid360.yaml"]
            ),
            description="Mid-360 到 LaserScan 的适配参数",
        ),
        DeclareLaunchArgument(
            "mid360_imu_params_file",
            default_value=PathJoinSubstitution(
                [kiwi_slam_share, "config", "mid360_imu.yaml"]
            ),
            description="Mid-360 IMU EKF 适配参数",
        ),
    ]

    pace = GroupAction(
        condition=IfCondition(start_lidar),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("bluesea2"), "launch", "uart_lidar.launch"]
                    )
                ),
                condition=LaunchConfigurationEquals(lidar_type, "pace"),
                launch_arguments={"params_file": pace_params}.items(),
            )
        ],
    )

    mid360 = GroupAction(
        condition=IfCondition(start_lidar),
        actions=[
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_lidar_publisher",
                condition=LaunchConfigurationEquals(lidar_type, "mid360"),
                output="screen",
                parameters=[mid360_params, {"user_config_path": mid360_network}],
            ),
            Node(
                package="kiwi_navigation",
                executable="kiwi_livox_scan_adapter",
                name="livox_scan_adapter",
                condition=LaunchConfigurationEquals(lidar_type, "mid360"),
                output="screen",
                parameters=[mid360_scan_params],
            ),
            Node(
                package="kiwi_slam",
                executable="kiwi_mid360_imu_adapter",
                name="mid360_imu_adapter",
                condition=LaunchConfigurationEquals(lidar_type, "mid360"),
                output="screen",
                parameters=[mid360_imu_params],
            ),
        ],
    )

    return LaunchDescription(arguments + [pace, mid360])
