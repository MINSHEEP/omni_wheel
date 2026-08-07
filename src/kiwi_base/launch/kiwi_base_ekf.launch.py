#!/usr/bin/env python3
"""启动 Kiwi 底盘和使用 C 板 IMU 的平面 EKF。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """底盘硬件与 EKF 分别读取 YAML，切换传感器不会改动底盘参数。"""
    default_base_params = PathJoinSubstitution(
        [FindPackageShare("kiwi_base"), "config", "kiwi_base.yaml"]
    )
    default_ekf_params = PathJoinSubstitution(
        [FindPackageShare("kiwi_base"), "config", "ekf_chassis.yaml"]
    )
    base_params = LaunchConfiguration("base_params_file")
    ekf_params = LaunchConfiguration("ekf_params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "base_params_file",
                default_value=default_base_params,
                description="底盘硬件、话题和结构参数",
            ),
            DeclareLaunchArgument(
                "ekf_params_file",
                default_value=default_ekf_params,
                description="EKF 输入源和融合参数",
            ),
            Node(
                package="kiwi_base",
                executable="kiwi_base_node",
                name="kiwi_base_node",
                output="screen",
                parameters=[base_params],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_params],
            ),
        ]
    )
