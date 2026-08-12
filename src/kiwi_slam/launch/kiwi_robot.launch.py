#!/usr/bin/env python3
"""启动 Kiwi 底盘、平面 EKF 和由 Xacro 生成的 RobotModel。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """底盘与 EKF 共用统一 YAML，结构尺寸由 Xacro 从该文件读取。"""
    package_share = FindPackageShare("kiwi_slam")
    base_params = LaunchConfiguration("base_params_file")
    start_base = LaunchConfiguration("start_base")
    start_ekf = LaunchConfiguration("start_ekf")
    start_description = LaunchConfiguration("start_description")

    xacro_file = PathJoinSubstitution([package_share, "urdf", "kiwi.urdf.xacro"])
    robot_description = Command(
        ["xacro ", xacro_file, " hardware_config:=", base_params]
    )

    arguments = [
        DeclareLaunchArgument(
            "base_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("kiwi_base"), "config", "kiwi_base.yaml"]
            ),
            description="底盘硬件、话题、测量和结构参数 YAML",
        ),
        DeclareLaunchArgument(
            "start_base", default_value="true", description="是否启动 USB 底盘节点"
        ),
        DeclareLaunchArgument(
            "start_ekf", default_value="true", description="是否启动平面 EKF"
        ),
        DeclareLaunchArgument(
            "start_description", default_value="true", description="是否发布 URDF 和静态 TF"
        ),
    ]

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            condition=IfCondition(start_description),
            output="screen",
            parameters=[base_params, {"robot_description": robot_description}],
        ),
        Node(
            package="kiwi_base",
            executable="kiwi_base_node",
            name="kiwi_base_node",
            condition=IfCondition(start_base),
            output="screen",
            parameters=[base_params],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            condition=IfCondition(start_ekf),
            output="screen",
            parameters=[base_params],
        ),
    ]
    return LaunchDescription(arguments + nodes)
