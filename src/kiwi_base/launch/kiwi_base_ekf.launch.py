#!/usr/bin/env python3
"""启动 Kiwi 底盘和平面 EKF；传感器静态 TF 统一由 URDF 发布。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
# 过渡期临时 TF 已停用，保留导入注释方便追溯，不再创建独立 TF 节点。
# from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """两个节点读取同一 YAML；IMU 和雷达安装关系由 robot_state_publisher 发布。"""
    default_params = PathJoinSubstitution(
        [FindPackageShare("kiwi_base"), "config", "kiwi_base.yaml"]
    )
    params_file = LaunchConfiguration("params_file")
    # 以下过渡参数已停用。正式系统只允许 URDF/Xacro 维护传感器静态 TF。
    # publish_imu_tf = LaunchConfiguration("publish_imu_static_tf")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="底盘和 EKF 共用的参数文件",
            ),
            # 过渡代码保留但不执行，避免与 URDF 重复发布 imu_link：
            # DeclareLaunchArgument(
            #     "publish_imu_static_tf",
            #     default_value="false",
            #     description="已停用：传感器 TF 由 URDF/Xacro 管理",
            # ),
            # Node(
            #     package="tf2_ros",
            #     executable="static_transform_publisher",
            #     name="imu_static_tf",
            #     condition=IfCondition(publish_imu_tf),
            #     arguments=[
            #         "--x", "0", "--y", "0", "--z", "0",
            #         "--roll", "0", "--pitch", "0", "--yaw", "0",
            #         "--frame-id", "base_footprint",
            #         "--child-frame-id", "imu_link",
            #     ],
            # ),
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
