#!/usr/bin/env python3
"""启动 E110、SLAM Toolbox、Kiwi 底盘和 Nav2 全向导航。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """使用 E110 的标准 /scan 和底盘统一参数文件启动二维建图导航。"""
    navigation_share = FindPackageShare("kiwi_navigation")
    slam_share = FindPackageShare("kiwi_slam")
    nav2_params = LaunchConfiguration("nav2_params_file")
    lidar_params = LaunchConfiguration("lidar_params_file")
    slam_params = LaunchConfiguration("slam_params_file")
    monitor_params = LaunchConfiguration("monitor_params_file")
    base_params = LaunchConfiguration("base_params_file")
    rviz_config = LaunchConfiguration("rviz_config_file")
    start_base = LaunchConfiguration("start_base")
    start_ekf = LaunchConfiguration("start_ekf")
    start_lidar = LaunchConfiguration("start_lidar")
    start_slam = LaunchConfiguration("start_slam")
    start_monitor = LaunchConfiguration("start_monitor")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_software_rendering = LaunchConfiguration("rviz_software_rendering")

    arguments = [
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=PathJoinSubstitution([navigation_share, "config", "nav2.yaml"]),
            description="Nav2、DWB、costmap 和 AMCL 参数 YAML",
        ),
        DeclareLaunchArgument(
            "lidar_params_file",
            default_value=PathJoinSubstitution(
                [slam_share, "config", "ls_m10p_uart.yaml"]
            ),
            description="镭神 M10P 串口和 LaserScan 参数 YAML",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution([slam_share, "config", "slam_toolbox.yaml"]),
            description="SLAM Toolbox 参数 YAML",
        ),
        DeclareLaunchArgument(
            "monitor_params_file",
            default_value=PathJoinSubstitution([slam_share, "config", "monitor.yaml"]),
            description="SLAM 链路诊断参数 YAML",
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
            description="二维建图导航 RViz 配置",
        ),
        DeclareLaunchArgument("start_base", default_value="true"),
        DeclareLaunchArgument("start_ekf", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("start_monitor", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_software_rendering", default_value="true"),
    ]

    robot_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_share, "launch", "kiwi_robot.launch.py"])
        ),
        launch_arguments={
            "base_params_file": base_params,
            "start_base": start_base,
            "start_ekf": start_ekf,
            "start_description": "true",
        }.items(),
    )

    # 复用 kiwi_slam 的 M10P 封装，避免三个入口各自复制节点定义；
    # 旧 bluesea2 只保留在源码树供回退，任何入口不再启动它。
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_share, "launch", "m10p_uart_lidar.launch.py"])
        ),
        condition=IfCondition(start_lidar),
        launch_arguments={"params_file": lidar_params}.items(),
    )

    slam = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        condition=IfCondition(start_slam),
        output="screen",
        parameters=[slam_params],
    )

    monitor = Node(
        package="kiwi_slam",
        executable="kiwi_slam_monitor",
        name="kiwi_slam_monitor",
        condition=IfCondition(start_monitor),
        output="screen",
        parameters=[monitor_params],
    )

    # 后方无传感器：把 bt_navigator 的行为树重写为无 BackUp 版本（路径按本机
    # kiwi_navigation share 解析）。移除 behavior_server 的 backup 插件后，默认
    # 行为树会因等不到 backup action server 而导致 bt_navigator 激活失败。
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
        arguments + [robot_stack, lidar, slam, monitor, navigation, rviz]
    )
