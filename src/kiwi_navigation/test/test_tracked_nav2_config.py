"""履带通过独立底盘 YAML 覆盖公共 Nav2 参数的契约。"""

import importlib.util
import math
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE.parent


def load_yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def load_platform():
    path = PACKAGE / "launch" / "platform.launch.py"
    spec = importlib.util.spec_from_file_location("platform_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_tracked_navigation_is_differential_and_uses_laser_imu():
    platform = load_platform()
    shared = load_yaml(PACKAGE / "config" / "nav2.yaml")
    tracked = load_yaml(
        SOURCE_ROOT / "kiwi_base" / "config" / "tracked_pwm.yaml"
    )
    nav = platform._nav2_for_chassis(shared, tracked, "tracked_pwm")
    base = tracked["tracked_base_node"]["ros__parameters"]
    ekf = tracked["ekf_filter_node"]["ros__parameters"]
    gate = tracked["kiwi_mapping_scan_gate"]["ros__parameters"]

    assert nav["amcl"]["ros__parameters"]["robot_model_type"] == (
        "nav2_amcl::DifferentialMotionModel"
    )
    assert "alpha5" not in nav["amcl"]["ros__parameters"]
    assert ekf["odom0"] == gate["source_odom_topic"] == "/chassis/odom/laser"
    assert ekf["imu0"] == gate["imu_topic"] == "/chassis/imu/data_raw"
    assert gate["require_imu_samples"] and gate["require_source_odom"]
    assert "/chassis/odom/raw" not in (ekf["odom0"], gate["source_odom_topic"])

    controller = nav["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = nav["velocity_smoother"]["ros__parameters"]
    assert controller["min_vel_y"] == controller["max_vel_y"] == 0.0
    assert controller["acc_lim_y"] == controller["decel_lim_y"] == 0.0
    assert controller["vy_samples"] == 1
    assert smoother["max_velocity"][1] == smoother["min_velocity"][1] == 0.0
    assert smoother["max_accel"][1] == smoother["max_decel"][1] == 0.0
    assert smoother["feedback"] == "OPEN_LOOP"
    assert controller["max_vel_x"] == smoother["max_velocity"][0]
    assert controller["max_vel_x"] <= base["max_linear_speed_m_s"]
    assert controller["max_vel_theta"] == smoother["max_velocity"][2]
    assert controller["max_vel_theta"] <= base["max_angular_speed_rad_s"]
    assert smoother["velocity_timeout"] <= base["cmd_vel_timeout_s"]
    smoother_params = nav["smoother_server"]["ros__parameters"]
    assert (
        smoother_params["constrained_smoother"]["minimum_turning_radius"]
        == 0.0
    )

    model = tracked["tracked_robot_model"]["ros__parameters"]
    radius_needed = math.hypot(
        model["base_length_m"] / 2, model["base_width_m"] / 2
    )
    for scope in ("local_costmap", "global_costmap"):
        params = nav[scope][scope]["ros__parameters"]
        assert params["robot_radius"] > radius_needed
        assert (
            params["inflation_layer"]["inflation_radius"]
            > params["robot_radius"]
        )


def test_kiwi_keeps_its_existing_nav2_parameters():
    platform = load_platform()
    shared = load_yaml(PACKAGE / "config" / "nav2.yaml")
    base = load_yaml(SOURCE_ROOT / "kiwi_base" / "config" / "kiwi_base.yaml")
    assert platform._nav2_for_chassis(shared, base, "kiwi") == shared
    assert shared["amcl"]["ros__parameters"]["robot_model_type"] == (
        "nav2_amcl::OmniMotionModel"
    )


def test_tracked_mapping_uses_rf2o_and_imu_without_motor_calibration(tmp_path):
    platform = load_platform()
    tracked = load_yaml(
        SOURCE_ROOT / "kiwi_base" / "config" / "tracked_pwm.yaml"
    )
    base = tracked["tracked_base_node"]["ros__parameters"]
    base["mapping_sensors_verified"] = True
    assert base["hardware_calibrated"] is False
    assert base["enable_motion"] is False
    config = tmp_path / "tracked_mapping.yaml"
    config.write_text(
        yaml.safe_dump(tracked, allow_unicode=True), encoding="utf-8"
    )

    context = LaunchContext()
    context.launch_configurations.update({
        "chassis": "tracked_pwm", "lidar": "m10p", "imu": "chassis",
        "odom": "auto", "mode": "mapping", "map": "",
        "base_params_file": str(config), "lidar_params_file": "",
        "nav2_params_file": "", "slam_params_file": "",
        "mapping_gate_params_file": "", "external_imu_topic": "",
        "external_odom_topic": "", "start_base": "true",
        "start_lidar": "false", "use_rviz": "false",
    })
    actions = platform._setup(context)
    executables = {action.node_executable for action in actions
                   if hasattr(action, "node_executable")}
    assert {"tracked_base_node", "rf2o_laser_odometry_node", "ekf_node",
            "kiwi_mapping_scan_gate", "sync_slam_toolbox_node"} <= executables
    # 手动建图不启动 Nav2，也不需要命令到 m/s 的换算。
    assert not any(action.__class__.__name__ == "IncludeLaunchDescription"
                   for action in actions)

    context.launch_configurations["mode"] = "localization"
    with pytest.raises(ValueError, match="导航须关闭开环试车"):
        platform._setup(context)
