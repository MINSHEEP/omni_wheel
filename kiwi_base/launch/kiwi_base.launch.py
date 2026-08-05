#!/usr/bin/env python3
"""启动 Kiwi 底盘节点，并为参数文件提供包内默认路径。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """使用已安装包中的参数文件；params_file 可由外部覆盖。"""
    default_params = PathJoinSubstitution(
        [FindPackageShare("kiwi_base"), "config", "kiwi_base.yaml"]
    )
    params_file = LaunchConfiguration("params_file")
    device = LaunchConfiguration("device")
    log_serial_frames = LaunchConfiguration("log_serial_frames")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Kiwi 节点参数文件，默认使用包内 config/kiwi_base.yaml",
            ),
            DeclareLaunchArgument(
                "device",
                default_value="/dev/robomaster_c",
                description="USB CDC 设备路径",
            ),
            DeclareLaunchArgument(
                "log_serial_frames",
                default_value="false",
                description="是否在终端逐帧打印 USB TX/RX 十六进制数据",
            ),
            Node(
                package="kiwi_base",
                executable="kiwi_base_node",
                name="kiwi_base_node",
                output="screen",
                parameters=[
                    params_file,
                    {"device": device, "log_serial_frames": log_serial_frames},
                ],
            ),
        ]
    )
