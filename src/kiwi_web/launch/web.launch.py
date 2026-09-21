"""Portable Web entry; optional overrides reach the owning ROS packages."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from kiwi_web.config import PARAMETER_FILES

OVERRIDES = ("workspace", "maps_dir", "host", "port", *PARAMETER_FILES)


def setup(context):
    arguments = ["--config", LaunchConfiguration("config").perform(context)]
    for key in OVERRIDES:
        value = LaunchConfiguration(key).perform(context)
        if value:
            arguments.extend(["--" + key.replace("_", "-"), value])
    return [
        Node(package="kiwi_web", executable="kiwi_web_server", output="screen", arguments=arguments)
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("kiwi_web"), "config", "web.yaml"]
                ),
            ),
            *[DeclareLaunchArgument(key, default_value="") for key in OVERRIDES],
            OpaqueFunction(function=setup),
        ]
    )
