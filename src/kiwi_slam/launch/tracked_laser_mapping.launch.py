#!/usr/bin/env python3
"""双履带 USB 通信 + M10P + RF2O 单一激光里程计 + SLAM Toolbox。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from kiwi_slam.runtime_contract import (
    validate_tracked_laser_parameters,
    validate_workspace_overlay,
)


def _validate_runtime_contract(context):
    validate_workspace_overlay(required_packages=("kiwi_base", "rf2o_laser_odometry"))
    validate_tracked_laser_parameters(
        LaunchConfiguration("base_params_file").perform(context),
        LaunchConfiguration("lidar_params_file").perform(context),
        LaunchConfiguration("rf2o_params_file").perform(context),
        LaunchConfiguration("mapping_gate_params_file").perform(context),
        LaunchConfiguration("slam_params_file").perform(context),
    )
    return []


def generate_launch_description():
    slam_share = FindPackageShare("kiwi_slam")
    base_params = LaunchConfiguration("base_params_file")
    lidar_params = LaunchConfiguration("lidar_params_file")
    rf2o_params = LaunchConfiguration("rf2o_params_file")
    gate_params = LaunchConfiguration("mapping_gate_params_file")
    slam_params = LaunchConfiguration("slam_params_file")
    rviz_config = LaunchConfiguration("rviz_config_file")

    arguments = [
        DeclareLaunchArgument(
            "base_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("kiwi_base"), "config", "tracked_laser.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "lidar_params_file",
            default_value=PathJoinSubstitution(
                [slam_share, "config", "ls_m10p_uart.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "rf2o_params_file",
            default_value=PathJoinSubstitution([slam_share, "config", "tracked_rf2o.yaml"]),
        ),
        DeclareLaunchArgument(
            "mapping_gate_params_file",
            default_value=PathJoinSubstitution(
                [slam_share, "config", "tracked_mapping_gate.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution([slam_share, "config", "slam_toolbox.yaml"]),
        ),
        DeclareLaunchArgument(
            "monitor_params_file",
            default_value=PathJoinSubstitution([slam_share, "config", "monitor.yaml"]),
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution([slam_share, "config", "kiwi_slam.rviz"]),
        ),
        DeclareLaunchArgument("start_base", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("start_odom", default_value="true"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("start_monitor", default_value="true"),
        DeclareLaunchArgument("start_description", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_software_rendering", default_value="true"),
    ]

    robot_description = Command([
        "xacro ", PathJoinSubstitution([slam_share, "urdf", "tracked.urdf.xacro"]),
        " hardware_config:=", base_params,
    ])
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_share, "launch", "m10p_uart_lidar.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("start_lidar")),
        launch_arguments={"params_file": lidar_params}.items(),
    )
    nodes = [
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="screen",
            condition=IfCondition(LaunchConfiguration("start_description")),
            parameters=[base_params, {"robot_description": robot_description}],
        ),
        Node(
            package="kiwi_base", executable="tracked_base_node",
            name="tracked_base_node", output="screen",
            condition=IfCondition(LaunchConfiguration("start_base")),
            parameters=[base_params],
        ),
        Node(
            package="rf2o_laser_odometry", executable="rf2o_laser_odometry_node",
            name="rf2o_laser_odometry", output="screen",
            condition=IfCondition(LaunchConfiguration("start_odom")),
            parameters=[rf2o_params],
        ),
        Node(
            package="kiwi_slam", executable="kiwi_mapping_scan_gate",
            name="kiwi_mapping_scan_gate", output="screen",
            condition=IfCondition(LaunchConfiguration("start_slam")),
            parameters=[gate_params],
        ),
        Node(
            package="slam_toolbox", executable="sync_slam_toolbox_node",
            name="slam_toolbox", output="screen",
            condition=IfCondition(LaunchConfiguration("start_slam")),
            parameters=[slam_params],
        ),
        Node(
            package="kiwi_slam", executable="kiwi_slam_monitor",
            name="kiwi_slam_monitor", output="screen",
            condition=IfCondition(LaunchConfiguration("start_monitor")),
            parameters=[LaunchConfiguration("monitor_params_file")],
        ),
        Node(
            package="rviz2", executable="rviz2", output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            arguments=["-d", rviz_config],
            additional_env={
                "QT_QPA_PLATFORM": "xcb",
                "LIBGL_ALWAYS_SOFTWARE": IfElseSubstitution(
                    LaunchConfiguration("rviz_software_rendering"), "1", "0"
                ),
            },
        ),
    ]
    return LaunchDescription(
        arguments + [OpaqueFunction(function=_validate_runtime_contract), lidar] + nodes
    )
