#!/usr/bin/env python3
"""Start the Kiwi base, E110 driver, TF model, SLAM Toolbox, and diagnostics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """直接启动底盘和 EKF，避免复用带过渡期静态 TF 的旧 launch。"""
    package_share = FindPackageShare("kiwi_slam")
    lidar_params = LaunchConfiguration("lidar_params_file")
    slam_params = LaunchConfiguration("slam_params_file")
    base_params = LaunchConfiguration("base_params_file")
    monitor_params = LaunchConfiguration("monitor_params_file")
    rviz_config = LaunchConfiguration("rviz_config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    start_base = LaunchConfiguration("start_base")
    start_lidar = LaunchConfiguration("start_lidar")
    start_slam = LaunchConfiguration("start_slam")
    start_monitor = LaunchConfiguration("start_monitor")
    start_description = LaunchConfiguration("start_description")
    use_rviz = LaunchConfiguration("use_rviz")

    xacro_file = PathJoinSubstitution([package_share, "urdf", "kiwi.urdf.xacro"])
    robot_description = Command(
        [
            "xacro ", xacro_file,
            " base_z:=", LaunchConfiguration("base_z"),
            " imu_x:=", LaunchConfiguration("imu_x"),
            " imu_y:=", LaunchConfiguration("imu_y"),
            " imu_z:=", LaunchConfiguration("imu_z"),
            " imu_roll:=", LaunchConfiguration("imu_roll"),
            " imu_pitch:=", LaunchConfiguration("imu_pitch"),
            " imu_yaw:=", LaunchConfiguration("imu_yaw"),
            " lidar_x:=", LaunchConfiguration("lidar_x"),
            " lidar_y:=", LaunchConfiguration("lidar_y"),
            " lidar_z:=", LaunchConfiguration("lidar_z"),
            " lidar_roll:=", LaunchConfiguration("lidar_roll"),
            " lidar_pitch:=", LaunchConfiguration("lidar_pitch"),
            " lidar_yaw:=", LaunchConfiguration("lidar_yaw"),
        ]
    )

    arguments = [
        DeclareLaunchArgument(
            "lidar_params_file",
            default_value=PathJoinSubstitution([package_share, "config", "lds_e110.yaml"]),
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution([package_share, "config", "slam_toolbox.yaml"]),
        ),
        DeclareLaunchArgument(
            "monitor_params_file",
            default_value=PathJoinSubstitution([package_share, "config", "monitor.yaml"]),
        ),
        DeclareLaunchArgument(
            "base_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("kiwi_base"), "config", "kiwi_base.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [package_share, "config", "kiwi_slam.rviz"]
            ),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_base", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("start_monitor", default_value="true"),
        DeclareLaunchArgument("start_description", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("base_z", default_value="0.0"),
        DeclareLaunchArgument("imu_x", default_value="0.0"),
        DeclareLaunchArgument("imu_y", default_value="0.0"),
        DeclareLaunchArgument("imu_z", default_value="0.0"),
        DeclareLaunchArgument("imu_roll", default_value="0.0"),
        DeclareLaunchArgument("imu_pitch", default_value="0.0"),
        DeclareLaunchArgument("imu_yaw", default_value="0.0"),
        DeclareLaunchArgument("lidar_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="0.20"),
        DeclareLaunchArgument("lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
    ]

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("bluesea2"), "launch", "uart_lidar.launch"])
        ),
        condition=IfCondition(start_lidar),
        launch_arguments={"params_file": lidar_params}.items(),
    )

    # 不 include kiwi_base_ekf.launch.py；IMU、雷达和机体固定 TF 只由下方 Xacro 发布。
    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            condition=IfCondition(start_description),
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
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
            condition=IfCondition(start_base),
            output="screen",
            parameters=[base_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="slam_toolbox",
            executable="sync_slam_toolbox_node",
            name="slam_toolbox",
            condition=IfCondition(start_slam),
            output="screen",
            parameters=[slam_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="kiwi_slam",
            executable="kiwi_slam_monitor",
            name="kiwi_slam_monitor",
            condition=IfCondition(start_monitor),
            output="screen",
            parameters=[monitor_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            condition=IfCondition(use_rviz),
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ]
    return LaunchDescription(arguments + [lidar] + nodes)
