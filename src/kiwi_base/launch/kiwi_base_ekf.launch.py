#!/usr/bin/env python3
"""启动活动底盘及其里程计链路；履带需要 RF2O、M10P 和 IMU。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from kiwi_platform import load_profile, validate_profile
from kiwi_platform.launching import robot_actions


def _setup(context):
    selector = LaunchConfiguration("platform_file").perform(context)
    overrides = {"base": LaunchConfiguration("params_file").perform(context)}
    profile = load_profile(selector, overrides, required_keys=("base",))
    if profile.odometry == "rf2o":
        profile = load_profile(selector, overrides,
                               required_keys=("base", "urdf", "lidar", "rf2o"))
    validate_profile(profile, "driver")
    tracked = profile.chassis == "tracked_pwm"
    return robot_actions(profile, description=tracked, lidar=tracked)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("platform_file", default_value=""),
        DeclareLaunchArgument("params_file", default_value=""),
        OpaqueFunction(function=_setup),
    ])
