#!/usr/bin/env python3
"""启动 M10P、已知地图定位和 Nav2 导航。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """使用 M10P 的标准 /scan 启动 AMCL 与 Nav2。"""
    navigation_share = FindPackageShare("kiwi_navigation")
    slam_share = FindPackageShare("kiwi_slam")
    map_file = LaunchConfiguration("map")
    nav2_params = LaunchConfiguration("nav2_params_file")
    lidar_params = LaunchConfiguration("lidar_params_file")
    base_params = LaunchConfiguration("base_params_file")
    rviz_config = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_software_rendering = LaunchConfiguration("rviz_software_rendering")

    arguments = [
        DeclareLaunchArgument("map", description="已保存地图的绝对 YAML 路径"),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=PathJoinSubstitution([navigation_share, "config", "nav2.yaml"]),
        ),
        DeclareLaunchArgument(
            "lidar_params_file",
            default_value=PathJoinSubstitution(
                [slam_share, "config", "ls_m10p_uart.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "base_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("kiwi_base"), "config", "kiwi_base.yaml"]
            ),
            description="底盘和 EKF 共用参数 YAML",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [navigation_share, "config", "navigation.rviz"]
            ),
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_software_rendering", default_value="true"),
    ]

    robot_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_share, "launch", "kiwi_robot.launch.py"])
        ),
        launch_arguments={
            "base_params_file": base_params,
            "start_base": "true",
            "start_ekf": "true",
            "start_description": "true",
        }.items(),
    )

    # 复用 kiwi_slam 的 M10P 封装；本入口历史上无 start_lidar 开关，保持原接口。
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_share, "launch", "m10p_uart_lidar.launch.py"])
        ),
        launch_arguments={"params_file": lidar_params}.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "localization_launch.py"]
            )
        ),
        launch_arguments={
            "map": map_file,
            "use_sim_time": "false",
            "autostart": "true",
            "params_file": nav2_params,
            "use_composition": "False",
            "use_respawn": "False",
            "log_level": "info",
        }.items(),
    )

    # 同 mapping 入口：注入无 BackUp 行为树，避免 bt_navigator 因缺 backup
    # action server 激活失败；后方补传感器后与 behavior_server 一起恢复。
    nav2_params_no_backup = RewrittenYaml(
        source_file=nav2_params,
        root_key="",
        param_rewrites={
            "default_nav_to_pose_bt_xml": PathJoinSubstitution(
                [navigation_share, "behavior_trees", "navigate_to_pose_no_backup.xml"]
            ),
            "default_nav_through_poses_bt_xml": PathJoinSubstitution(
                [
                    navigation_share,
                    "behavior_trees",
                    "navigate_through_poses_no_backup.xml",
                ]
            ),
        },
        convert_types=True,
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": "false",
            "autostart": "true",
            "params_file": nav2_params_no_backup,
            "use_composition": "False",
            "use_respawn": "False",
            "log_level": "info",
        }.items(),
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

    return LaunchDescription(
        arguments + [robot_stack, lidar, localization, navigation, rviz]
    )
