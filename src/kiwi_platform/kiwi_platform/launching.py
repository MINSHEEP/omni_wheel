"""各独立 launch 文件共用的节点编排，参数文件只来自活动底盘档案。"""
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from .profile import validate_lidar, validate_rf2o


def validate_map_file(value):
    """在启动底盘及 Nav2 前确认地图 YAML 和图像均可读取。"""
    requested = Path(value).expanduser()
    if not requested.is_absolute():
        raise ValueError("map 必须是已保存地图的 YAML 文件绝对路径")
    map_file = requested.resolve()
    if not map_file.is_file() or map_file.suffix != ".yaml":
        raise ValueError("map 必须是已保存地图的 YAML 文件绝对路径")
    try:
        config = yaml.safe_load(map_file.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"无法读取地图 YAML：{map_file}: {error}") from error
    image = config.get("image") if isinstance(config, dict) else None
    if not isinstance(image, str) or not image.strip():
        raise ValueError(f"地图 YAML 缺少 image：{map_file}")
    image_file = Path(image).expanduser()
    if not image_file.is_absolute():
        image_file = map_file.parent / image_file
    if not image_file.is_file():
        raise ValueError(f"地图图像文件不存在：{image_file}")
    return str(map_file)


def robot_actions(profile, *, base=True, ekf=True, description=True, lidar=True):
    actions = []
    if description:
        robot_description = Command([
            "xacro ", profile.path("urdf"), " hardware_config:=", profile.path("base")
        ])
        actions.append(Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="screen",
            parameters=[profile.path("base"), {"robot_description": robot_description}],
        ))
    if base:
        actions.append(Node(
            package="kiwi_base", executable=profile.driver, name=profile.driver,
            output="screen", parameters=[profile.path("base")],
        ))
    if lidar:
        validate_lidar(profile)
        actions.append(Node(
            package="lslidar_driver", executable="lslidar_driver_node",
            name="lslidar_driver_node", output="screen", parameters=[profile.path("lidar")],
        ))
    if ekf:
        if profile.odometry == "rf2o":
            validate_rf2o(profile)
            try:
                get_package_share_directory("rf2o_laser_odometry")
            except Exception as error:
                raise ValueError("缺少 rf2o_laser_odometry；先导入 tools/platform_dependencies.repos 并构建") from error
            actions.append(Node(
                package="rf2o_laser_odometry", executable="rf2o_laser_odometry_node",
                name="rf2o_laser_odometry", output="screen",
                parameters=[profile.path("rf2o")],
            ))
            actions.append(Node(
                package="kiwi_slam", executable="rf2o_covariance_adapter",
                name="rf2o_covariance_adapter", output="screen",
                parameters=[profile.path("rf2o")],
            ))
        actions.append(Node(
            package="robot_localization", executable="ekf_node", name="ekf_filter_node",
            output="screen", parameters=[profile.path("base")],
        ))
    return actions


def mapping_actions(profile, *, monitor=False, monitor_map_is_static=None):
    actions = [
        Node(package="kiwi_slam", executable="kiwi_mapping_scan_gate",
             name="kiwi_mapping_scan_gate", output="screen", parameters=[profile.path("gate")]),
        Node(package="slam_toolbox", executable="sync_slam_toolbox_node",
             name="slam_toolbox", output="screen", parameters=[profile.path("slam")]),
    ]
    if monitor:
        parameters = [profile.path("monitor")]
        if monitor_map_is_static is not None:
            parameters.append({"map_is_static": monitor_map_is_static})
        actions.append(Node(package="kiwi_slam", executable="kiwi_slam_monitor",
                            name="kiwi_slam_monitor", output="screen", parameters=parameters))
    return actions


def nav2_parameters(profile):
    share = Path(get_package_share_directory("kiwi_navigation"))
    return RewrittenYaml(
        source_file=profile.path("nav2"), root_key="", convert_types=True,
        param_rewrites={
            "default_nav_to_pose_bt_xml": str(share / "behavior_trees/navigate_to_pose_no_backup.xml"),
            "default_nav_through_poses_bt_xml": str(share / "behavior_trees/navigate_through_poses_no_backup.xml"),
        },
    )


def nav2_include(name, *, params, map_file=None):
    share = Path(get_package_share_directory("nav2_bringup"))
    arguments = {
        "params_file": params, "use_sim_time": "false", "autostart": "true",
        "use_composition": "False", "use_respawn": "False", "log_level": "info",
    }
    if map_file is not None:
        arguments["map"] = map_file
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / name)),
        launch_arguments=arguments.items(),
    )
