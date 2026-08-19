#!/usr/bin/env python3
"""镭神 M10P 串口雷达的唯一启动封装，供建图/定位/导航三个入口复用。

驱动可执行文件创建的是普通 rclcpp::Node（lslidar_driver_node.cpp 的 main），
因此这里用普通 Node 动作启动，不引入生命周期管理；上游示例的 LifecycleNode
写法对该驱动没有实际状态机支撑。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """按 params_file 启动唯一的 M10P lslidar_driver_node 实例。"""
    params_file = LaunchConfiguration("params_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("kiwi_slam"), "config", "ls_m10p_uart.yaml"]
                ),
                description="镭神 M10P 串口与 LaserScan 参数 YAML",
            ),
            # 节点名与参数 YAML 顶层键 lslidar_driver_node 保持一致；不加命名空间，
            # laserscan_topic 已是绝对名 /scan，与 SLAM/Nav2 合约直接对齐。
            Node(
                package="lslidar_driver",
                executable="lslidar_driver_node",
                name="lslidar_driver_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
