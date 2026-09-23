#!/usr/bin/env python3
"""统一平台入口：底盘、雷达、惯导、里程计、建图/定位独立选择。"""

from __future__ import annotations

import math
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from kiwi_slam.runtime_contract import (
    validate_base_parameters,
    validate_workspace_overlay,
)


def _config(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _choose(value: str, name: str, choices: set[str]) -> str:
    if value not in choices:
        raise ValueError(
            f"{name}={value!r} 无效，可选：{', '.join(sorted(choices))}"
        )
    return value


def _load(path: Path) -> dict:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"无法读取配置 {path}: {error}") from error
    if not isinstance(data, dict):
        raise ValueError(f"配置必须是 YAML 映射：{path}")
    return data


def _merge(target: dict, overlay: dict) -> None:
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _merge(target[key], value)
        else:
            target[key] = value


def _materialize(data: dict, prefix: str) -> str:
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        prefix=f"{prefix}_",
        suffix=".yaml",
        delete=False,
    ) as output:
        yaml.safe_dump(data, output, allow_unicode=True, sort_keys=False)
        return output.name


def _include(package: str, launch_file: str, arguments: dict | None = None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory(package))
                / "launch"
                / launch_file
            )
        ),
        launch_arguments=(arguments or {}).items(),
    )


def _setup(context):
    chassis = _choose(
        _config(context, "chassis"), "chassis", {"kiwi", "tracked_pwm"}
    )
    lidar = _choose(
        _config(context, "lidar"), "lidar", {"m10p", "external_scan"}
    )
    imu = _choose(_config(context, "imu"), "imu", {"chassis", "external"})
    odom = _choose(
        _config(context, "odom"), "odom", {"auto", "wheel", "rf2o", "external"}
    )
    mode = _choose(
        _config(context, "mode"), "mode", {"driver", "mapping", "localization"}
    )
    odom = (
        ("wheel" if chassis == "kiwi" else "rf2o") if odom == "auto" else odom
    )
    if chassis == "kiwi" and odom != "wheel":
        raise ValueError(
            "Kiwi 当前只支持 wheel 里程计；请为新增来源单独建立配置"
        )
    if chassis == "tracked_pwm" and odom == "wheel":
        raise ValueError("履带无电机反馈，不能选择 wheel 里程计")

    validate_workspace_overlay(
        anchor_package="kiwi_navigation",
        required_packages=("kiwi_base", "kiwi_slam"),
    )
    base_share = Path(get_package_share_directory("kiwi_base"))
    slam_share = Path(get_package_share_directory("kiwi_slam"))
    nav_share = Path(get_package_share_directory("kiwi_navigation"))
    base_file = (
        Path(
            _config(context, "base_params_file")
            or str(
                base_share
                / "config"
                / (
                    "kiwi_base.yaml"
                    if chassis == "kiwi"
                    else "tracked_pwm.yaml"
                )
            )
        )
        .expanduser()
        .resolve()
    )
    base_config = _load(base_file)
    if chassis == "kiwi":
        validate_base_parameters(base_file)
    else:
        tracked = base_config.get("tracked_base_node", {}).get(
            "ros__parameters", {}
        )
        if not isinstance(tracked, dict):
            raise ValueError("tracked_base_node.ros__parameters 缺失")
        calibrated = tracked.get("hardware_calibrated") is True
        motion_enabled = tracked.get("enable_motion") is True
        if motion_enabled and not calibrated:
            raise ValueError("enable_motion=true 需要先完成硬件标定")
        if calibrated:
            for key in (
                "track_separation_m",
                "command_per_m_s",
                "max_motor_command",
            ):
                value = tracked.get(key)
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (float, int))
                    or not math.isfinite(value)
                    or value <= 0
                ):
                    raise ValueError(f"履带 {key} 必须是标定后的正数")
        if mode != "driver" and not calibrated:
            raise ValueError(
                "履带几何/命令/传感器外参仍为 TODO；完成标定并设置 hardware_calibrated=true 后才能导航"
            )
        if mode != "driver" and not motion_enabled:
            raise ValueError(
                "履带 enable_motion=false；完成空载方向验证后才能导航"
            )
        if (
            odom == "external"
            and mode != "driver"
            and not _config(context, "external_odom_topic")
        ):
            raise ValueError("odom=external 必须设置 external_odom_topic")

    if imu == "external":
        external_imu = _config(context, "external_imu_topic")
        if not external_imu:
            raise ValueError("imu=external 必须设置 external_imu_topic")
        base_config["ekf_filter_node"]["ros__parameters"][
            "imu0"
        ] = external_imu
        base_file = Path(_materialize(base_config, "platform_base"))
    if odom == "external":
        base_config["ekf_filter_node"]["ros__parameters"]["odom0"] = _config(
            context, "external_odom_topic"
        )
        base_file = Path(_materialize(base_config, "platform_base"))

    xacro_file = (
        slam_share
        / "urdf"
        / ("kiwi.urdf.xacro" if chassis == "kiwi" else "tracked.urdf.xacro")
    )
    robot_description = Command(
        ["xacro ", str(xacro_file), " hardware_config:=", str(base_file)]
    )
    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
    ]
    if _config(context, "start_base") == "true":
        actions.append(
            Node(
                package="kiwi_base",
                executable=(
                    "kiwi_base_node"
                    if chassis == "kiwi"
                    else "tracked_base_node"
                ),
                name=(
                    "kiwi_base_node"
                    if chassis == "kiwi"
                    else "tracked_base_node"
                ),
                output="screen",
                parameters=[str(base_file)],
            )
        )
    if mode == "driver":
        return actions

    if odom == "rf2o":
        # RF2O 只发布激光里程计消息；odom->base_footprint 由 EKF 独占发布。
        try:
            get_package_share_directory("rf2o_laser_odometry")
        except Exception as error:
            raise ValueError(
                "缺少 rf2o_laser_odometry；"
                "先导入 tools/platform_dependencies.repos 并构建"
            ) from error
        actions.append(
            Node(
                package="rf2o_laser_odometry",
                executable="rf2o_laser_odometry_node",
                name="rf2o_laser_odometry",
                output="screen",
                parameters=[
                    {
                        "laser_scan_topic": "/scan",
                        "odom_topic": "/chassis/odom/laser",
                        "publish_tf": False,
                        "base_frame_id": "base_footprint",
                        "odom_frame_id": "odom",
                        "freq": 20.0,
                    }
                ],
            )
        )
    actions.append(
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[str(base_file)],
        )
    )
    if lidar == "m10p" and _config(context, "start_lidar") == "true":
        actions.append(
            _include(
                "kiwi_slam",
                "m10p_uart_lidar.launch.py",
                {
                    "params_file": _config(context, "lidar_params_file")
                    or str(slam_share / "config/ls_m10p_uart.yaml")
                },
            )
        )

    nav_file = Path(
        _config(context, "nav2_params_file")
        or str(nav_share / "config/nav2.yaml")
    )
    nav_data = _load(nav_file)
    if chassis == "tracked_pwm":
        _merge(nav_data, _load(nav_share / "config/tracked_nav2_overlay.yaml"))
    nav_file = Path(_materialize(nav_data, "platform_nav2"))
    bt_params = RewrittenYaml(
        source_file=str(nav_file),
        root_key="",
        convert_types=True,
        param_rewrites={
            "default_nav_to_pose_bt_xml": str(
                nav_share / "behavior_trees/navigate_to_pose_no_backup.xml"
            ),
            "default_nav_through_poses_bt_xml": str(
                nav_share
                / "behavior_trees/navigate_through_poses_no_backup.xml"
            ),
        },
    )
    if mode == "mapping":
        gate_file = _config(context, "mapping_gate_params_file") or str(
            slam_share
            / "config"
            / (
                "mapping_gate.yaml"
                if chassis == "kiwi"
                else "tracked_mapping_gate.yaml"
            )
        )
        actions.extend(
            [
                Node(
                    package="kiwi_slam",
                    executable="kiwi_mapping_scan_gate",
                    name="kiwi_mapping_scan_gate",
                    output="screen",
                    parameters=[gate_file],
                ),
                Node(
                    package="slam_toolbox",
                    executable="sync_slam_toolbox_node",
                    name="slam_toolbox",
                    output="screen",
                    parameters=[
                        _config(context, "slam_params_file")
                        or str(slam_share / "config/slam_toolbox.yaml")
                    ],
                ),
            ]
        )
    else:
        map_file = _config(context, "map")
        if not map_file or not Path(map_file).is_file():
            raise ValueError(
                "mode=localization 必须提供存在的 map:=绝对路径.yaml"
            )
        actions.append(
            _include(
                "nav2_bringup",
                "localization_launch.py",
                {
                    "map": map_file,
                    "params_file": str(nav_file),
                    "use_sim_time": "false",
                    "autostart": "true",
                    "use_composition": "False",
                    "use_respawn": "False",
                },
            )
        )
    actions.append(
        _include(
            "nav2_bringup",
            "navigation_launch.py",
            {
                "params_file": bt_params,
                "use_sim_time": "false",
                "autostart": "true",
                "use_composition": "False",
                "use_respawn": "False",
            },
        )
    )
    if _config(context, "use_rviz") == "true":
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", str(nav_share / "config/navigation.rviz")],
            )
        )
    return actions


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("chassis", default_value="kiwi"),
        DeclareLaunchArgument("lidar", default_value="m10p"),
        DeclareLaunchArgument("imu", default_value="chassis"),
        DeclareLaunchArgument("odom", default_value="auto"),
        DeclareLaunchArgument("mode", default_value="mapping"),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("base_params_file", default_value=""),
        DeclareLaunchArgument("lidar_params_file", default_value=""),
        DeclareLaunchArgument("nav2_params_file", default_value=""),
        DeclareLaunchArgument("slam_params_file", default_value=""),
        DeclareLaunchArgument("mapping_gate_params_file", default_value=""),
        DeclareLaunchArgument("external_imu_topic", default_value=""),
        DeclareLaunchArgument("external_odom_topic", default_value=""),
        DeclareLaunchArgument("start_base", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_setup)])
