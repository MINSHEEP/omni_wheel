#!/usr/bin/env python3
"""活动底盘、URDF、里程计和 IMU 的公共硬件链路。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from kiwi_platform import load_profile, validate_profile
from kiwi_platform.launching import robot_actions
from kiwi_slam.runtime_contract import validate_workspace_overlay


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
    overrides = {"base": _value(context, "base_params_file")}
    selected = load_profile(selector, overrides, required_keys=("base",))
    start_ekf = _bool(context, "start_ekf")
    keys = ["base", "urdf"]
    if selected.odometry == "rf2o" and start_ekf:
        keys += ["lidar", "rf2o"]
    profile = load_profile(selector, overrides, required_keys=tuple(keys))
    validate_profile(profile, "driver")
    lidar = _value(context, "start_lidar")
    if lidar not in ("", "true", "false"):
        raise ValueError("start_lidar 必须是 true 或 false")
    start_lidar = (profile.odometry == "rf2o") if not lidar else lidar == "true"
    if start_lidar and "lidar" not in profile.files:
        profile = load_profile(selector, overrides, required_keys=tuple(keys + ["lidar"]))
    return robot_actions(
        profile, base=_bool(context, "start_base"), ekf=start_ekf,
        description=_bool(context, "start_description"), lidar=start_lidar,
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("platform_file", default_value=""),
        DeclareLaunchArgument("base_params_file", default_value=""),
        DeclareLaunchArgument("start_base", default_value="true"),
        DeclareLaunchArgument("start_ekf", default_value="true"),
        DeclareLaunchArgument("start_description", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value=""),
        OpaqueFunction(function=_setup),
    ])
