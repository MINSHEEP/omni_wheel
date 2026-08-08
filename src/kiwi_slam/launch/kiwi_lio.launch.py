#!/usr/bin/env python3
"""启动 Kiwi 轮式里程计、Mid-360S 驱动和 FAST-LIO2 三维建图。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """保持轮式 odom；原始 IMU 给 FAST-LIO2，补协方差后的支路给平面 EKF。"""
    package_share = FindPackageShare("kiwi_slam")
    start_base = LaunchConfiguration("start_base")
    start_ekf = LaunchConfiguration("start_ekf")
    start_lidar = LaunchConfiguration("start_lidar")
    start_lio = LaunchConfiguration("start_lio")
    start_imu_adapter = LaunchConfiguration("start_imu_adapter")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_software_rendering = LaunchConfiguration("rviz_software_rendering")
    lidar_config = LaunchConfiguration("lidar_config_file")
    lidar_params = LaunchConfiguration("lidar_params_file")
    imu_params = LaunchConfiguration("imu_params_file")
    lio_config = LaunchConfiguration("lio_config_file")
    base_params = LaunchConfiguration("base_params_file")
    ekf_params = LaunchConfiguration("ekf_params_file")
    rviz_config = LaunchConfiguration("rviz_config_file")

    arguments = [
        DeclareLaunchArgument(
            "start_base", default_value="true", description="是否启动 USB 底盘节点"
        ),
        DeclareLaunchArgument(
            "start_ekf", default_value="true", description="是否启动 Mid-360S 平面 EKF"
        ),
        DeclareLaunchArgument(
            "start_lidar", default_value="true", description="是否启动 Livox ROS 驱动"
        ),
        DeclareLaunchArgument(
            "start_lio", default_value="true", description="是否启动 FAST-LIO2"
        ),
        DeclareLaunchArgument(
            "start_imu_adapter", default_value="true", description="是否生成 Livox EKF IMU 支路"
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="true", description="是否启动三维建图 RViz"
        ),
        DeclareLaunchArgument(
            "rviz_software_rendering", default_value="false", description="RViz 是否使用软件 OpenGL"
        ),
        DeclareLaunchArgument(
            "lidar_config_file",
            default_value=PathJoinSubstitution(
                [package_share, "config", "mid360s_network.json"]
            ),
            description="Livox SDK 网络 JSON",
        ),
        DeclareLaunchArgument(
            "lidar_params_file",
            default_value=PathJoinSubstitution(
                [package_share, "config", "livox_mid360s.yaml"]
            ),
            description="Livox ROS 驱动参数 YAML",
        ),
        DeclareLaunchArgument(
            "imu_params_file",
            default_value=PathJoinSubstitution(
                [package_share, "config", "mid360_imu.yaml"]
            ),
            description="Mid-360S IMU 适配参数 YAML",
        ),
        DeclareLaunchArgument(
            "lio_config_file",
            default_value=PathJoinSubstitution(
                [package_share, "config", "fast_lio_mid360s.yaml"]
            ),
            description="FAST-LIO2 参数 YAML",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [package_share, "config", "kiwi_lio.rviz"]
            ),
            description="三维建图 RViz 配置",
        ),
        DeclareLaunchArgument(
            "base_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("kiwi_base"), "config", "kiwi_base.yaml"]
            ),
            description="底盘硬件参数 YAML",
        ),
        DeclareLaunchArgument(
            "ekf_params_file",
            default_value=PathJoinSubstitution(
                [package_share, "config", "ekf_mid360.yaml"]
            ),
            description="Mid-360S 主路线 EKF 参数 YAML",
        ),
    ]

    # 只复用底盘公共层，三维 LIO 不依赖旧二维建图 launch。
    base_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "kiwi_robot.launch.py"])
        ),
        launch_arguments={
            "base_params_file": base_params,
            "ekf_params_file": ekf_params,
            "start_base": start_base,
            "start_ekf": start_ekf,
            "start_description": "true",
        }.items(),
    )

    # 不修改上游驱动文件；网络地址和 UDP 端口全部由本项目 JSON 提供。
    lidar_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        condition=IfCondition(start_lidar),
        output="screen",
        # 网络 JSON 的安装绝对路径由 launch 负责连接，其余驱动参数只读 YAML。
        parameters=[lidar_params, {"user_config_path": lidar_config}],
    )

    imu_adapter = Node(
        package="kiwi_slam",
        executable="kiwi_mid360_imu_adapter",
        name="mid360_imu_adapter",
        condition=IfCondition(start_imu_adapter),
        output="screen",
        parameters=[imu_params],
    )

    lio = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        condition=IfCondition(start_lio),
        output="screen",
        parameters=[lio_config],
        # FAST-LIO2 原生发布 camera_init->body；先隔离到调试 TF 话题，避免与
        # robot_localization 的 odom->base_footprint 形成第二套底盘 TF 所有权。
        remappings=[("/tf", "/fast_lio/tf")],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(use_rviz),
        output="screen",
        arguments=["-d", rviz_config],
        additional_env={
            "QT_QPA_PLATFORM": "xcb",
            "LIBGL_ALWAYS_SOFTWARE": IfElseSubstitution(
                rviz_software_rendering, "1", "0"
            ),
        },
    )

    return LaunchDescription(arguments + [base_stack, lidar_driver, imu_adapter, lio, rviz])
