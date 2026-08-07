"""检查底盘硬件 YAML 与 C 板 IMU EKF YAML 的接口契约。"""

from pathlib import Path

import yaml


def test_chassis_ekf_uses_wheel_twist_and_chassis_imu():
    """备用 EKF 使用轮速和 C 板姿态，不依赖 Mid-360S 包。"""
    config_dir = Path(__file__).resolve().parents[1] / "config"
    base = yaml.safe_load((config_dir / "kiwi_base.yaml").read_text(encoding="utf-8"))[
        "kiwi_base_node"
    ]["ros__parameters"]
    ekf = yaml.safe_load((config_dir / "ekf_chassis.yaml").read_text(encoding="utf-8"))[
        "ekf_filter_node"
    ]["ros__parameters"]

    assert base["wheel_odom_topic"] == ekf["odom0"] == "/chassis/odom/raw"
    assert base["imu_topic"] == ekf["imu0"] == "/chassis/imu/data_raw"
    assert base["feedback_debug_topic"] == "/chassis/feedback/raw"
    assert ekf["two_d_mode"] is True
    assert ekf["publish_tf"] is True
    assert ekf["world_frame"] == ekf["odom_frame"] == "odom"
    assert base["odom_frame_id"] == ekf["odom_frame"]
    assert base["base_frame_id"] == ekf["base_link_frame"] == "base_footprint"
    assert base["imu_frame_id"] == "chassis_imu_link"
    assert len(ekf["odom0_config"]) == 15
    assert [i for i, enabled in enumerate(ekf["odom0_config"]) if enabled] == [6, 7, 11]
    assert len(ekf["imu0_config"]) == 15
    assert [i for i, enabled in enumerate(ekf["imu0_config"]) if enabled] == [5, 11]


def test_chassis_imu_and_feedback_debug_are_always_published():
    """底盘节点始终保留 C 板标准 IMU 和 12 字段原始调试支路。"""
    source = (
        Path(__file__).resolve().parents[1] / "src" / "kiwi_base_node.cpp"
    ).read_text(encoding="utf-8")
    active_lines = [
        line for line in source.splitlines() if not line.lstrip().startswith("//")
    ]
    assert "publish_imu(state, angular_velocity, stamp);" in "\n".join(active_lines)
    assert "void publish_imu(" in source
    assert "publish_feedback_debug(state, stamp);" in "\n".join(active_lines)


def test_static_measurement_filters_match_observed_noise_and_ros_yaw_sign():
    """Z 轴翻转为逆时针正，静止死区覆盖本次采集到的 IMU 和轮速抖动。"""
    config_path = Path(__file__).resolve().parents[1] / "config" / "kiwi_base.yaml"
    base = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
        "kiwi_base_node"
    ]["ros__parameters"]

    assert base["imu_angular_velocity_sign"] == [1.0, 1.0, -1.0]
    assert base["imu_angular_velocity_deadband_rad_s"] == [0.015, 0.015, 0.015]
    assert base["wheel_twist_deadband"] == [0.015, 0.015, 0.022]


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


def test_yaml_is_the_single_source_for_tools_and_robot_structure():
    """键盘、测试节点和 Xacro 结构尺寸必须与底盘参数位于同一 YAML。"""
    config_path = Path(__file__).resolve().parents[1] / "config" / "kiwi_base.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    assert config["kiwi_teleop_keyboard"]["ros__parameters"]["speed_m_s"] > 0.0
    assert config["kiwi_odom_test"]["ros__parameters"]["odom_topic"] == "/odometry/filtered"
    model = config["kiwi_robot_model"]["ros__parameters"]
    assert model["base_radius_m"] > 0.0
    assert model["wheel_width_m"] > 0.0


def test_runtime_code_does_not_keep_hardware_parameter_defaults():
    """运行节点和串口工具不得再硬编码轮径、轮距、减速比或控制速度。"""
    package = Path(__file__).resolve().parents[1]
    node = (package / "src" / "kiwi_base_node.cpp").read_text(encoding="utf-8")
    teleop = (package / "src" / "kiwi_teleop_keyboard.cpp").read_text(encoding="utf-8")
    kinematics = (package / "include" / "kiwi_base" / "kiwi_kinematics.hpp").read_text(
        encoding="utf-8"
    )
    serial_test = (package / "tools" / "kiwi_serial_test.py").read_text(encoding="utf-8")

    assert "declare_required" in node
    assert 'declare_parameter<double>("speed_m_s")' in teleop
    assert "wheel_radius_m = 0.1015" not in kinematics
    assert "DEFAULT_WHEEL_RADIUS" not in serial_test
    assert 'default="/dev/robomaster_c"' not in serial_test


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
