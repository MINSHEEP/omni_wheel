#!/usr/bin/env python3
"""按活动底盘分别启动驱动、RF2O/轮式 EKF、SLAM 和诊断。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from kiwi_platform import load_profile, validate_profile
from kiwi_platform.profile import validate_monitor
from kiwi_platform.launching import robot_actions, mapping_actions
from kiwi_slam.runtime_contract import validate_workspace_overlay

PARAMS = {"base_params_file": "base", "lidar_params_file": "lidar", "slam_params_file": "slam",
          "mapping_gate_params_file": "gate", "monitor_params_file": "monitor"}


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _bool(context, name):
    value = _value(context, name)
    if value not in ("true", "false"):
        raise ValueError(f"{name} 必须是 true 或 false")
    return value == "true"


def _setup(context):
    validate_workspace_overlay(required_packages=("kiwi_base", "kiwi_platform"))
    selector = _value(context, "platform_file")
    overrides = {field: _value(context, argument) for argument, field in PARAMS.items()}
    selected = load_profile(selector, {"base": overrides["base"]}, required_keys=("base",))
    slam = _bool(context, "start_slam")
    keys = ["base", "urdf", "lidar", "monitor"]
    if selected.odometry == "rf2o":
        keys.append("rf2o")
    if slam:
        keys += ["gate", "slam"]
    profile = load_profile(selector, {key: value for key, value in overrides.items() if key in keys},
                           required_keys=tuple(keys))
    validate_profile(profile, "mapping" if slam else "driver")
    base, ekf, lidar = (_bool(context, field) for field in ("start_base", "start_ekf", "start_lidar"))
    if slam and not ekf:
        raise ValueError("建图必须启动所选里程计与 IMU EKF")
    actions = robot_actions(profile, base=base, ekf=ekf,
                            description=_bool(context, "start_description"), lidar=lidar)
    if slam:
        actions += mapping_actions(profile)
    if _bool(context, "start_monitor"):
        validate_monitor(profile)
        static = _value(context, "monitor_map_is_static")
        if static not in ("", "true", "false"):
            raise ValueError("monitor_map_is_static 必须是 true 或 false")
        parameters = [profile.path("monitor")]
        if slam and profile.odometry == "rf2o" and static != "true":
            guard = profile.data("base")["tracked_estimation_guard"]["ros__parameters"]
            gate = profile.data("gate")["kiwi_mapping_scan_gate"]["ros__parameters"]
            parameters.append({"mapping_gate_ready_topic": gate["gate_ready_topic"],
                               "estimation_state_topic": guard["state_topic"],
                               "inertial_valid_topic": profile.data("base")["tracked_inertial_odometry"]["ros__parameters"]["valid_topic"],
                               "inertial_reason_topic": profile.data("base")["tracked_inertial_odometry"]["ros__parameters"]["reason_topic"]})
        if static:
            parameters.append({"map_is_static": static == "true"})
        actions.append(Node(package="kiwi_slam", executable="kiwi_slam_monitor",
                            name="kiwi_slam_monitor", output="screen", parameters=parameters))
    if _bool(context, "use_rviz"):
        config = _value(context, "rviz_config_file") or str(Path(get_package_share_directory("kiwi_slam")) / "config/kiwi_slam.rviz")
        actions.append(Node(package="rviz2", executable="rviz2", output="screen",
                            arguments=["-d", config], additional_env={
                                "QT_QPA_PLATFORM": "xcb",
                                "LIBGL_ALWAYS_SOFTWARE": "1" if _bool(context, "rviz_software_rendering") else "0",
                            }))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("platform_file", default_value=""),
        *[DeclareLaunchArgument(field, default_value="") for field in PARAMS],
        DeclareLaunchArgument("rviz_config_file", default_value=""),
        DeclareLaunchArgument("monitor_map_is_static", default_value=""),
        *[DeclareLaunchArgument(field, default_value=value) for field, value in (
            ("start_base", "true"), ("start_ekf", "true"), ("start_lidar", "true"),
            ("start_slam", "true"), ("start_monitor", "true"),
            ("start_description", "true"), ("use_rviz", "true"),
            ("rviz_software_rendering", "false"),
        )],
        OpaqueFunction(function=_setup),
    ])
