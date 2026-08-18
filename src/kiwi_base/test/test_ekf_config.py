"""检查底盘、EKF 和键盘共享同一 YAML 的接口契约。"""

from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE / "config" / "kiwi_base.yaml"


def load_config():
    return yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))


def test_chassis_ekf_uses_wheel_twist_and_chassis_imu_from_one_yaml():
    config = load_config()
    base = config["kiwi_base_node"]["ros__parameters"]
    ekf = config["ekf_filter_node"]["ros__parameters"]

    assert base["wheel_odom_topic"] == ekf["odom0"] == "/chassis/odom/raw"
    assert base["imu_topic"] == ekf["imu0"] == "/chassis/imu/data_raw"
    assert base["feedback_debug_topic"] == "/chassis/feedback/raw"
    # 新 C 板固件（2026-08-14 起）发送 54 字节 CRC 封装帧。
    assert base["feedback_framed"] is True
    assert ekf["two_d_mode"] is True
    assert ekf["publish_tf"] is True
    assert ekf["world_frame"] == ekf["odom_frame"] == "odom"
    assert base["odom_frame_id"] == ekf["odom_frame"]
    assert base["base_frame_id"] == ekf["base_link_frame"] == "base_footprint"
    assert base["imu_frame_id"] == "chassis_imu_link"
    assert [i for i, enabled in enumerate(ekf["odom0_config"]) if enabled] == [6, 7, 11]
    # 2026-08-18: C 板 yaw 零漂已控制，启用 yaw(索引5) 和 vyaw(索引11) 融合
    assert [i for i, enabled in enumerate(ekf["imu0_config"]) if enabled] == [5, 11]
    assert base["imu_publish_orientation"] is True


def test_keyboard_and_base_use_direct_cmd_vel_interface():
    config = load_config()
    base = config["kiwi_base_node"]["ros__parameters"]
    keyboard = config["kiwi_teleop_keyboard"]["ros__parameters"]

    assert base["cmd_vel_topic"] == keyboard["cmd_vel_topic"] == "/cmd_vel"


def test_keyboard_speeds_do_not_exceed_base_limits():
    config = load_config()
    base = config["kiwi_base_node"]["ros__parameters"]
    keyboard = config["kiwi_teleop_keyboard"]["ros__parameters"]

    # 唯一限幅入口在 kiwi_base_node；键盘期望速度超限只会被静默截断，视为配置错误。
    assert 0.0 < keyboard["speed_m_s"] <= base["max_linear_speed_m_s"]
    assert keyboard["speed_m_s"] <= base["max_lateral_speed_m_s"]
    assert 0.0 < keyboard["yaw_rad_s"] <= base["max_angular_speed_rad_s"]


def test_chassis_imu_and_feedback_debug_are_always_published():
    source = (PACKAGE / "src" / "kiwi_base_node.cpp").read_text(encoding="utf-8")
    active_lines = [
        line for line in source.splitlines() if not line.lstrip().startswith("//")
    ]
    active_source = "\n".join(active_lines)
    assert "publish_imu(state, angular_velocity, stamp);" in active_source
    assert "void publish_imu(" in source
    assert "publish_feedback_debug(state, stamp);" in active_source
    assert "pending_feedback_" in source
    assert "queue_raw_feedback" in source


def test_static_measurement_filters_match_observed_noise_and_ros_yaw_sign():
    base = load_config()["kiwi_base_node"]["ros__parameters"]

    assert base["imu_angular_velocity_sign"] == [1.0, 1.0, -1.0]
    assert base["imu_angular_velocity_scale"] == [1.0, 1.0, 1.69]
    # z 轴阈值 2026-08-14 调整到 0.050 以完全覆盖温漂范围
    assert base["imu_angular_velocity_deadband_rad_s"] == [0.015, 0.015, 0.050]
    assert base["wheel_twist_deadband"] == [0.015, 0.015, 0.022]
    assert base["motor_feedback_deadband_rad_s"] == [0.20, 0.20, 0.20]


def test_gyro_bias_filter_parameters_are_consistent():
    base = load_config()["kiwi_base_node"]["ros__parameters"]

    # 上电静止标定至少要有一个样本；EMA 与静止确认时间必须为正。
    assert base["imu_gyro_calibration_samples"] >= 1
    assert base["imu_gyro_bias_time_constant_s"] > 0.0
    assert base["imu_stationary_hold_s"] > 0.0
    # 零漂上限须能覆盖死区级别的零偏，否则钳制会低估真实零漂。
    assert base["imu_gyro_max_bias_rad_s"] >= max(
        base["imu_angular_velocity_deadband_rad_s"]
    )
    # 低通截止频率必须低于反馈采样率的奈奎斯特频率。
    assert 0.0 < base["imu_gyro_lowpass_cutoff_hz"] < base["publish_rate"] / 2.0


def test_filter_inputs_have_positive_covariances():
    base = load_config()["kiwi_base_node"]["ros__parameters"]
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


def test_yaml_is_the_single_source_for_runtime_and_robot_structure():
    config = load_config()
    assert config["kiwi_teleop_keyboard"]["ros__parameters"]["speed_m_s"] > 0.0
    assert config["kiwi_odom_test"]["ros__parameters"]["odom_topic"] == "/odometry/filtered"
    model = config["kiwi_robot_model"]["ros__parameters"]
    assert model["base_radius_m"] > 0.0
    assert model["wheel_width_m"] > 0.0
    assert not (PACKAGE / "config" / "ekf_chassis.yaml").exists()


def test_runtime_code_does_not_keep_hardware_parameter_defaults():
    node = (PACKAGE / "src" / "kiwi_base_node.cpp").read_text(encoding="utf-8")
    teleop = (PACKAGE / "src" / "kiwi_teleop_keyboard.cpp").read_text(encoding="utf-8")
    kinematics = (PACKAGE / "include" / "kiwi_base" / "kiwi_kinematics.hpp").read_text(
        encoding="utf-8"
    )
    serial_test = (PACKAGE / "tools" / "kiwi_serial_test.py").read_text(encoding="utf-8")

    assert "declare_required" in node
    assert 'declare_parameter<double>("speed_m_s")' in teleop
    assert "wheel_radius_m = 0.1015" not in kinematics
    assert "DEFAULT_WHEEL_RADIUS" not in serial_test
    assert 'default="/dev/robomaster_c"' not in serial_test


def test_keyboard_loads_installed_yaml_when_no_parameter_file_is_given():
    teleop = (PACKAGE / "src" / "kiwi_teleop_keyboard.cpp").read_text(encoding="utf-8")

    assert 'get_package_share_directory("kiwi_base")' in teleop
    assert "arguments_with_default_params" in teleop
    assert 'arguments.emplace_back("--ros-args")' in teleop
    assert 'argument == "--params-file"' in teleop
    assert '"/config/kiwi_base.yaml"' in teleop


def test_ekf_launch_passes_one_parameter_file_to_both_nodes():
    launch = (PACKAGE / "launch" / "kiwi_base_ekf.launch.py").read_text(encoding="utf-8")
    active_lines = [
        line for line in launch.splitlines() if not line.lstrip().startswith("#")
    ]
    active_launch = "\n".join(active_lines)
    assert 'LaunchConfiguration("params_file")' in launch
    assert active_launch.count("parameters=[params_file]") == 2
    assert "ekf_params_file" not in active_launch
    assert "static_transform_publisher" not in active_launch


def test_feedback_debug_message_keeps_the_decoded_0x3b_field_order():
    message_path = PACKAGE / "msg" / "FeedbackDebug.msg"
    lines = [
        line.split("#", 1)[0].rstrip()
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
