#!/usr/bin/env python3
"""启动 Kiwi 底盘、镭神 M10P 雷达、SLAM Toolbox、模型和诊断。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """复用公共底盘层，构成可按模块启停的 M10P 二维建图链路。"""
    package_share = FindPackageShare("kiwi_slam")
    lidar_params = LaunchConfiguration("lidar_params_file")
    slam_params = LaunchConfiguration("slam_params_file")
    base_params = LaunchConfiguration("base_params_file")
    monitor_params = LaunchConfiguration("monitor_params_file")
    mapping_gate_params = LaunchConfiguration("mapping_gate_params_file")
    rviz_config = LaunchConfiguration("rviz_config_file")

    start_base = LaunchConfiguration("start_base")
    start_ekf = LaunchConfiguration("start_ekf")
    start_lidar = LaunchConfiguration("start_lidar")
    start_slam = LaunchConfiguration("start_slam")
    start_monitor = LaunchConfiguration("start_monitor")
    start_description = LaunchConfiguration("start_description")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_software_rendering = LaunchConfiguration("rviz_software_rendering")

    arguments = [
        DeclareLaunchArgument(
            "lidar_params_file",
            default_value=PathJoinSubstitution(
                [package_share, "config", "ls_m10p_uart.yaml"]
            ),
            description="镭神 M10P 串口和 LaserScan 参数 YAML",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution([package_share, "config", "slam_toolbox.yaml"]),
            description="SLAM Toolbox 二维建图参数 YAML",
        ),
        DeclareLaunchArgument(
            "monitor_params_file",
            default_value=PathJoinSubstitution([package_share, "config", "monitor.yaml"]),
            description="二维链路诊断参数 YAML",
        ),
        DeclareLaunchArgument(
            "mapping_gate_params_file",
            default_value=PathJoinSubstitution(
                [package_share, "config", "mapping_gate.yaml"]
            ),
            description="SLAM 首帧标定、静止和扫描一致性门控参数",
        ),
        DeclareLaunchArgument(
            "base_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("kiwi_base"), "config", "kiwi_base.yaml"]
            ),
            description="底盘硬件参数 YAML",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [package_share, "config", "kiwi_slam.rviz"]
            ),
            description="Kiwi M10P 二维建图 RViz 配置",
        ),
        DeclareLaunchArgument(
            "start_base", default_value="true", description="是否启动 USB 底盘节点"
        ),
        DeclareLaunchArgument(
            "start_ekf", default_value="true", description="是否启动反馈里程计平面 EKF"
        ),
        DeclareLaunchArgument(
            "start_lidar", default_value="true", description="是否启动镭神 M10P 驱动"
        ),
        DeclareLaunchArgument(
            "start_slam", default_value="true", description="是否启动 SLAM Toolbox"
        ),
        DeclareLaunchArgument(
            "start_monitor", default_value="true", description="是否启动二维链路诊断节点"
        ),
        DeclareLaunchArgument(
            "start_description", default_value="true", description="是否发布 URDF 和静态 TF"
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="false", description="是否启动二维建图 RViz"
        ),
        # 工控机显卡驱动会触发 RViz Map 的 GLSL sampler 错误，默认只对 RViz 使用软件渲染。
        DeclareLaunchArgument(
            "rviz_software_rendering",
            default_value="true",
            description="RViz 是否使用软件 OpenGL",
        ),
    ]

    # 雷达统一走 kiwi_slam 内的 M10P 封装，保证 /scan 只有一个发布者。
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "m10p_uart_lidar.launch.py"])
        ),
        condition=IfCondition(start_lidar),
        launch_arguments={"params_file": lidar_params}.items(),
    )

    robot_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "kiwi_robot.launch.py"])
        ),
        launch_arguments={
            "base_params_file": base_params,
            "start_base": start_base,
            "start_ekf": start_ekf,
            "start_description": start_description,
        }.items(),
    )

    # 二维建图层只保留 M10P、SLAM、诊断和 RViz；底盘层由 kiwi_robot.launch.py 管理。
    nodes = [
        Node(
            package="kiwi_slam",
            executable="kiwi_mapping_scan_gate",
            name="kiwi_mapping_scan_gate",
            condition=IfCondition(start_slam),
            output="screen",
            parameters=[mapping_gate_params],
        ),
        Node(
            package="slam_toolbox",
            executable="sync_slam_toolbox_node",
            name="slam_toolbox",
            condition=IfCondition(start_slam),
            output="screen",
            parameters=[slam_params],
        ),
        Node(
            package="kiwi_slam",
            executable="kiwi_slam_monitor",
            name="kiwi_slam_monitor",
            condition=IfCondition(start_monitor),
            output="screen",
            parameters=[monitor_params],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            condition=IfCondition(use_rviz),
            output="screen",
            arguments=["-d", rviz_config],
            # 不设置 ROS name，避免 RViz 内部 TF 节点被同一个 __node remap 重命名。
            additional_env={
                "QT_QPA_PLATFORM": "xcb",
                "LIBGL_ALWAYS_SOFTWARE": IfElseSubstitution(
                    rviz_software_rendering, "1", "0"
                ),
            },
        ),
    ]
    return LaunchDescription(arguments + [robot_stack, lidar] + nodes)
