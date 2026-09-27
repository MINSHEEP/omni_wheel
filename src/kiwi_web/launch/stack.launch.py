"""Owned robot stack with Web-only velocity topics for each sensor pipeline."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from kiwi_web.config import PARAMETER_FILES, MID360_PARAMETER_FILES, MID360_OPTIONS
from kiwi_platform import load_profile, validate_profile
from kiwi_platform.launching import validate_map_file


def setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in ("mapping", "navigation", "remote"):
        raise ValueError("Unsupported web stack mode")
    fields = {"base_params_file": "base", "lidar_params_file": "lidar",
              "slam_params_file": "slam", "mapping_gate_params_file": "gate",
              "monitor_params_file": "monitor", "nav2_params_file": "nav2"}
    selector = LaunchConfiguration("platform_file").perform(context)
    base_override = context.launch_configurations.get("base_params_file", "")
    selected = load_profile(selector, {"base": base_override}, required_keys=("base",))
    pipeline = LaunchConfiguration("sensor_pipeline", default="auto").perform(context)
    if pipeline == "auto":
        pipeline = "mid360s" if selected.chassis == "tracked_pwm" else "legacy"
    if pipeline not in ("legacy", "mid360s"):
        raise ValueError("未知 Web 传感器链路")
    profile = (load_profile(selector,
                            {field: context.launch_configurations.get(key, "")
                             for key, field in fields.items()})
               if pipeline == "legacy" else selected)
    expected = LaunchConfiguration("expected_chassis").perform(context)
    if expected and expected != profile.chassis:
        raise ValueError("Web 服务启动后活动底盘已变化，请重启 Web 服务")
    if pipeline == "mid360s" and profile.chassis != "tracked_pwm":
        raise ValueError("MID360S 网页链路仅支持 tracked_pwm 履带底盘")
    if pipeline == "legacy":
        validate_profile(profile, "mapping" if mode == "mapping" else "localization" if mode == "navigation" else "driver")
    else:
        for unused in ("lidar_params_file", "mapping_gate_params_file", "monitor_params_file"):
            if context.launch_configurations.get(unused):
                raise ValueError(f"MID360S 链路不使用旧参数 {unused}")
    map_file = validate_map_file(LaunchConfiguration("map").perform(context)) if mode == "navigation" else ""
    slam = Path(get_package_share_directory("kiwi_slam"))
    navigation = Path(get_package_share_directory("kiwi_navigation"))
    if pipeline == "mid360s":
        pipeline_args = {
            "mode": "mapping" if mode == "mapping" else "localization",
            "platform_file": str(profile.selector),
            "motion_gate_input_topic": "/kiwi_web/cmd_vel",
        }
        for key in ("base_params_file", *MID360_PARAMETER_FILES,
                    "lidar_interface", "host_ip", "lidar_ip", "lidar_model", "map_pcd"):
            value = context.launch_configurations.get(key, "")
            if value:
                pipeline_args[key] = value
        actions = [GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(slam / "launch/mid360_pipeline.launch.py")),
            launch_arguments=pipeline_args.items(),
        )], forwarding=False)]
        if mode == "mapping":
            slam_file = (context.launch_configurations.get("slam_params_file")
                         or str(slam / "config/mid360_slam_toolbox.yaml"))
            if not Path(slam_file).is_file():
                raise ValueError("MID360S 建图参数文件不存在")
            actions.append(Node(package="slam_toolbox", executable="sync_slam_toolbox_node",
                                name="slam_toolbox", output="screen", parameters=[slam_file]))
        elif mode == "navigation":
            nav_args = {
                "map": map_file,
                "platform_file": str(profile.selector),
                "sensor_pipeline": "mid360s",
                "raw_cmd_vel_topic": "/kiwi_web/nav_raw",
                "cmd_vel_topic": "/kiwi_web/nav_cmd_vel",
            }
            for key in ("base_params_file", "nav2_params_file"):
                value = context.launch_configurations.get(key, "")
                if value:
                    nav_args[key] = value
            actions.append(GroupAction([IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(navigation / "launch/navigation_core.launch.py")),
                launch_arguments=nav_args.items(),
            )], forwarding=False))
        return actions
    # Omit empty overrides: the owning package supplies its own defaults.
    overrides = {
        key: context.launch_configurations.get(key, "")
        for key in PARAMETER_FILES
        if key != "nav2_params_file"
    }
    overrides = {key: value for key, value in overrides.items() if value}
    actions = [
        GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(slam / "launch/kiwi_slam.launch.py")),
                    launch_arguments={
                        "platform_file": str(profile.selector),
                        "use_rviz": "false",
                        "start_slam": str(mode == "mapping").lower(),
                        "start_monitor": str(mode != "remote").lower(),
                        "monitor_map_is_static": str(mode == "navigation").lower(),
                        **overrides,
                    }.items(),
                )
            ],
            forwarding=False,
        )
    ]
    if mode != "navigation":
        return actions
    arguments = {
        "map": map_file,
        "platform_file": str(profile.selector),
        "raw_cmd_vel_topic": "/kiwi_web/nav_raw",
        "cmd_vel_topic": "/kiwi_web/nav_cmd_vel",
    }
    if context.launch_configurations.get("base_params_file"):
        arguments["base_params_file"] = context.launch_configurations["base_params_file"]
    if context.launch_configurations.get("lidar_params_file"):
        arguments["lidar_params_file"] = context.launch_configurations["lidar_params_file"]
    if context.launch_configurations.get("nav2_params_file"):
        arguments["nav2_params_file"] = context.launch_configurations["nav2_params_file"]
    actions.append(
        GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(navigation / "launch/navigation_core.launch.py")
                    ),
                    launch_arguments=arguments.items(),
                )
            ],
            forwarding=False,
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="remote"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("platform_file", default_value=""),
            DeclareLaunchArgument("expected_chassis", default_value=""),
            *[DeclareLaunchArgument(key, default_value="") for key in PARAMETER_FILES],
            *[DeclareLaunchArgument(key, default_value="") for key in MID360_PARAMETER_FILES],
            *[DeclareLaunchArgument(key, default_value="auto" if key == "sensor_pipeline" else "")
              for key in MID360_OPTIONS],
            OpaqueFunction(function=setup),
        ]
    )
