"""检查单一 YAML 中的轮里程计和 EKF 接口没有发生错配。"""

from pathlib import Path

import yaml


def test_ekf_fuses_planar_velocity_with_relative_imu_yaw_and_yaw_rate():
    """EKF 使用轮速 vx/vy，并用相对 Euler yaw 约束 gyro_z 的长期积分漂移。"""
    config_path = Path(__file__).resolve().parents[1] / "config" / "kiwi_base.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    base = config["kiwi_base_node"]["ros__parameters"]
    ekf = config["ekf_filter_node"]["ros__parameters"]

    assert base["wheel_odom_topic"] == ekf["odom0"] == "/wheel/odom_raw"
    assert base["imu_topic"] == ekf["imu0"] == "/imu/data_raw"
    assert base["feedback_debug_topic"] == "/kiwi_base/feedback_debug"
    assert ekf["two_d_mode"] is True
    assert ekf["publish_tf"] is True
    assert ekf["world_frame"] == ekf["odom_frame"] == "odom"
    assert base["odom_frame_id"] == ekf["odom_frame"]
    assert base["base_frame_id"] == ekf["base_link_frame"] == "base_footprint"
    assert base["imu_frame_id"] == "imu_link"
    assert len(ekf["odom0_config"]) == len(ekf["imu0_config"]) == 15
    assert [i for i, enabled in enumerate(ekf["odom0_config"]) if enabled] == [6, 7]
    assert [i for i, enabled in enumerate(ekf["imu0_config"]) if enabled] == [5, 11]
    assert ekf["imu0_differential"] is False
    assert ekf["imu0_relative"] is True


def test_static_measurement_filters_match_observed_noise_and_ros_yaw_sign():
    """Z 轴翻转为逆时针正，静止死区覆盖本次采集到的 IMU 和轮速抖动。"""
    config_path = Path(__file__).resolve().parents[1] / "config" / "kiwi_base.yaml"
    base = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
        "kiwi_base_node"
    ]["ros__parameters"]

    assert base["imu_angular_velocity_sign"] == [1.0, 1.0, -1.0]
    assert base["imu_angular_velocity_deadband_rad_s"] == [0.015, 0.015, 0.015]
    assert base["wheel_twist_deadband"] == [0.015, 0.015, 0.015]


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


def test_ekf_launch_does_not_publish_temporary_sensor_tf():
    """正式启动文件不得执行过渡期的零位姿 IMU 静态变换。"""
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "kiwi_base_ekf.launch.py"
    launch = launch_path.read_text(encoding="utf-8")
    active_lines = [
        line for line in launch.splitlines() if not line.lstrip().startswith("#")
    ]
    active_launch = "\n".join(active_lines)
    assert "publish_imu_static_tf" not in active_launch
    assert "static_transform_publisher" not in active_launch


def test_feedback_debug_message_keeps_the_decoded_0x3b_field_order():
    """调试消息必须按 0x3B 顺序提供 12 个可直接绘图的标量字段。"""
    package_root = Path(__file__).resolve().parents[1]
    message_path = package_root / "msg" / "FeedbackDebug.msg"
    lines = [
        line
        for line in message_path.read_text(encoding="utf-8").splitlines()
        if line and not line.startswith("#")
    ]
    assert lines == [
        "std_msgs/Header header",
        "float32 motor_1_actual_rad_s",
        "float32 motor_2_actual_rad_s",
        "float32 motor_3_actual_rad_s",
        "float32 roll_rad",
        "float32 pitch_rad",
        "float32 yaw_rad",
        "float32 gyro_x_raw_rad_s",
        "float32 gyro_y_raw_rad_s",
        "float32 gyro_z_raw_rad_s",
        "float32 accel_x_m_s2",
        "float32 accel_y_m_s2",
        "float32 accel_z_m_s2",
    ]
