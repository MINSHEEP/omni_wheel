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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Kiwi 节点参数文件，默认使用包内 config/kiwi_base.yaml",
            ),
            Node(
                package="kiwi_base",
                executable="kiwi_base_node",
                name="kiwi_base_node",
                output="screen",
                parameters=[
                    params_file,
                ],
            ),
        ]
    )
