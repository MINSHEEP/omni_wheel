#!/usr/bin/env python3
"""启动 Kiwi 底盘、平面 EKF，以及临时的零位姿 IMU 静态变换。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """两个节点读取同一 YAML；URDF 接管 imu_link 后应关闭临时静态 TF。"""
    default_params = PathJoinSubstitution(
        [FindPackageShare("kiwi_base"), "config", "kiwi_base.yaml"]
    )
    params_file = LaunchConfiguration("params_file")
    publish_imu_tf = LaunchConfiguration("publish_imu_static_tf")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="底盘和 EKF 共用的参数文件",
            ),
            DeclareLaunchArgument(
                "publish_imu_static_tf",
                default_value="true",
                description="URDF 未发布 base_footprint 到 imu_link 时启用临时零变换",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="imu_static_tf",
                condition=IfCondition(publish_imu_tf),
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "base_footprint",
                    "--child-frame-id", "imu_link",
                ],
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
