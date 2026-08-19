#!/usr/bin/env python3
"""启动 Kiwi 底盘，以及融合电机反馈速度与异步 IMU yaw/gyro_z 的平面 EKF。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """底盘和 EKF 读取同一个 kiwi_base.yaml。"""
    default_params = PathJoinSubstitution(
        [FindPackageShare("kiwi_base"), "config", "kiwi_base.yaml"]
    )
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="底盘和 EKF 共用参数文件",
            ),
            Node(
                package="kiwi_base",
                executable="kiwi_base_node",
                name="kiwi_base_node",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
