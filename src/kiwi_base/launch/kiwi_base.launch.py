#!/usr/bin/env python3
"""只启动活动底盘对应的 USB 驱动。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from kiwi_platform import load_profile, validate_profile
from kiwi_platform.launching import robot_actions


def _setup(context):
    profile = load_profile(
        LaunchConfiguration("platform_file").perform(context),
        {"base": LaunchConfiguration("params_file").perform(context)},
        required_keys=("base",),
    )
    validate_profile(profile, "driver")
    return robot_actions(profile, ekf=False, description=False, lidar=False)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("platform_file", default_value=""),
        DeclareLaunchArgument("params_file", default_value=""),
        OpaqueFunction(function=_setup),
    ])
