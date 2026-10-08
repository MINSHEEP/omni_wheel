#!/usr/bin/env python3
"""履带 MID360S、FAST-LIO2 三维里程计与二维扫描的公共数据链。"""

import atexit
import json
import math
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

from kiwi_platform import load_profile
from kiwi_slam.mid360_contract import validate_mount
from kiwi_slam.mid360_network import driver_config, validate_network
from kiwi_slam.workspace_paths import default_map_pcd


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _bool(context, name):
    value = _value(context, name)
    if value not in ("true", "false"):
        raise ValueError(f"{name} 必须是 true 或 false")
    return value == "true"


def _setup(context):
    profile = load_profile(_value(context, "platform_file"),
                           {"base": _value(context, "base_params_file")},
                           required_keys=("base",))
    if profile.chassis != "tracked_pwm":
        raise ValueError("当前 MID360 链路仅适用于 tracked_pwm 履带底盘")
    share = Path(get_package_share_directory("kiwi_slam"))
    mount_file = _value(context, "mount_file") or str(share / "config/mid360_mount.yaml")
    fastlio_file = _value(context, "fastlio_params_file") or str(share / "config/mid360_fastlio.yaml")
    mount = validate_mount(mount_file, fastlio_file)
    mode = _value(context, "mode")
    if mode not in ("mapping", "localization"):
        raise ValueError("mode 必须是 mapping 或 localization")
    motion_input = _value(context, "motion_gate_input_topic")
    if not motion_input.startswith("/") or motion_input == "/mid360/cmd_vel_safe":
        raise ValueError("MID360 速度门控输入必须是独立的绝对话题")
    pcd_value = _value(context, "map_pcd")
    pcd = Path(pcd_value).expanduser() if pcd_value else default_map_pcd()
    if not pcd.is_absolute() or not pcd.parent.is_dir():
        raise ValueError("map_pcd 必须是父目录已存在的绝对 PCD 路径")
    z_min = float(_value(context, "scan_z_min"))
    z_max = float(_value(context, "scan_z_max"))
    if not math.isfinite(z_min) or not math.isfinite(z_max) or not 0 <= z_min < z_max:
        raise ValueError("扫描高度必须满足 0 <= scan_z_min < scan_z_max 且均为有限值")

    for package in ("fast_lio", "kiwi_base"):
        try:
            get_package_share_directory(package)
        except Exception as error:
            raise ValueError(f"缺少 {package}；请先安装并加载 MID360 工作区") from error

    mount_params = {
        "extrinsic_calibrated": True,
        "base_to_body_xyz": mount["base_to_body_xyz"],
        "base_to_body_rpy": mount["base_to_body_rpy"],
    }
    description = Command([
        "xacro ", str(share / "urdf/tracked_mid360.urdf.xacro"),
        " hardware_config:=", profile.path("base"), " mount_config:=", mount_file,
    ])
    actions = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", output="screen",
             parameters=[{"robot_description": description}]),
        # FAST-LIO2 的 camera_init 是本次启动的本地 odom 原点，仅作可视化别名。
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="mid360_odom_origin", output="screen",
             arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0",
                        "--pitch", "0", "--yaw", "0", "--frame-id", "odom",
                        "--child-frame-id", "camera_init"]),
        Node(package="fast_lio", executable="fastlio_mapping", name="fastlio_mapping",
             output="screen", parameters=[fastlio_file, {
                 "map_file_path": str(pcd),
                 "publish.map_en": mode == "mapping",
                 "pcd_save.pcd_save_en": mode == "mapping",
             }],
             # 移植版无关闭 TF 开关；隔离其 camera_init->body TF，由适配器独占公共 odom TF。
             remappings=[("/Odometry", "/fastlio/odometry"), ("/tf", "/fastlio/tf_private")]),
        Node(package="kiwi_slam", executable="lio_odom_adapter",
             name="lio_odom_adapter", output="screen", parameters=[{
                 "input_topic": "/fastlio/odometry", **mount_params,
             }]),
        Node(package="kiwi_slam", executable="mid360_scan_projector",
             name="mid360_scan_projector", output="screen", parameters=[{
                 "input_topic": "/cloud_registered_body",
                 "output_topic": "/scan_mid360",
                 "body_to_lidar_xyz": mount["body_to_lidar_xyz"],
                 "body_to_lidar_rpy": mount["body_to_lidar_rpy"],
                 "z_min": z_min, "z_max": z_max,
                 **mount_params,
             }]),
        Node(package="kiwi_slam", executable="mid360_motion_gate",
             name="mid360_motion_gate", output="screen", parameters=[{
                 "input_topic": _value(context, "motion_gate_input_topic"),
             }]),
    ]
    if _bool(context, "start_base"):
        actions.append(Node(package="kiwi_base", executable="tracked_base_node",
                            name="tracked_base_node", output="screen",
                            parameters=[profile.path("base"), {
                                "cmd_vel_topic": "/mid360/cmd_vel_safe",
                            }]))
    if _bool(context, "start_driver"):
        try:
            get_package_share_directory("livox_ros_driver2")
        except Exception as error:
            raise ValueError("缺少 livox_ros_driver2；请先编译并加载统一工作区") from error
        host, lidar = validate_network(_value(context, "lidar_interface"),
                                       _value(context, "host_ip"),
                                       _value(context, "lidar_ip"))
        fd, config_path = tempfile.mkstemp(prefix="kiwi-mid360-", suffix=".json")
        with os.fdopen(fd, "w", encoding="utf-8") as output:
            json.dump(driver_config(host, lidar, _value(context, "lidar_model")), output, indent=2)
        atexit.register(Path(config_path).unlink, missing_ok=True)
        actions.append(Node(package="livox_ros_driver2", executable="livox_ros_driver2_node",
                            name="livox_lidar_publisher", output="screen", parameters=[{
                                "xfer_format": 1, "multi_topic": 0, "data_src": 0,
                                "publish_freq": 10.0, "output_data_type": 0,
                                "frame_id": "lidar_link", "user_config_path": config_path,
                                "cmdline_input_bd_code": "livox0000000001",
                            }]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("platform_file", default_value=""),
        DeclareLaunchArgument("base_params_file", default_value=""),
        DeclareLaunchArgument("mount_file", default_value=""),
        DeclareLaunchArgument("fastlio_params_file", default_value=""),
        DeclareLaunchArgument("mode", default_value="mapping"),
        DeclareLaunchArgument("map_pcd", default_value="",
                              description="空值保存到工作区 maps/mid360_map.pcd"),
        DeclareLaunchArgument("start_base", default_value="true"),
        DeclareLaunchArgument("start_driver", default_value="true"),
        DeclareLaunchArgument("motion_gate_input_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("lidar_interface", default_value="enp2s0"),
        DeclareLaunchArgument("host_ip", default_value="192.168.1.50"),
        DeclareLaunchArgument("lidar_ip", default_value="192.168.1.117"),
        DeclareLaunchArgument("lidar_model", default_value="Mid360s"),
        DeclareLaunchArgument("scan_z_min", default_value="0.10"),
        DeclareLaunchArgument("scan_z_max", default_value="1.20"),
        OpaqueFunction(function=_setup),
    ])
