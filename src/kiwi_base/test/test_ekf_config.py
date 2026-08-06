"""检查单一 YAML 中的轮里程计和 EKF 接口没有发生错配。"""

from pathlib import Path

import yaml


def test_ekf_uses_only_independent_planar_measurements():
    """第一阶段只融合轮速 vx/vy 与 IMU gyro_z，不重复融合积分位姿。"""
    config_path = Path(__file__).resolve().parents[1] / "config" / "kiwi_base.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    base = config["kiwi_base_node"]["ros__parameters"]
    ekf = config["ekf_filter_node"]["ros__parameters"]

    assert base["wheel_odom_topic"] == ekf["odom0"] == "/wheel/odom_raw"
    assert base["imu_topic"] == ekf["imu0"] == "/imu/data_raw"
    assert ekf["two_d_mode"] is True
    assert ekf["publish_tf"] is True
    assert ekf["world_frame"] == ekf["odom_frame"] == "odom"
    assert base["odom_frame_id"] == ekf["odom_frame"]
    assert base["base_frame_id"] == ekf["base_link_frame"] == "base_footprint"
    assert base["imu_frame_id"] == "imu_link"
    assert len(ekf["odom0_config"]) == len(ekf["imu0_config"]) == 15
    assert [i for i, enabled in enumerate(ekf["odom0_config"]) if enabled] == [6, 7]
    assert [i for i, enabled in enumerate(ekf["imu0_config"]) if enabled] == [11]


def test_filter_inputs_have_positive_covariances():
    """被 EKF 使用的输入必须带非零方差。"""
    config_path = Path(__file__).resolve().parents[1] / "config" / "kiwi_base.yaml"
    base = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
        "kiwi_base_node"
    ]["ros__parameters"]

    names = [
        "wheel_pose_variance",
        "wheel_twist_variance",
        "imu_orientation_variance",
        "imu_angular_velocity_variance",
        "imu_linear_acceleration_variance",
    ]
    for name in names:
        assert len(base[name]) == 3
        assert all(value > 0.0 for value in base[name])
