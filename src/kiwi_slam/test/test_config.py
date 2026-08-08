"""Contract tests for E110, Mid-360S, SLAM, launch, and URDF integration files."""

import json
from pathlib import Path
import xml.etree.ElementTree as element_tree

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def load_parameters(name, node):
    data = yaml.safe_load((PACKAGE / "config" / name).read_text(encoding="utf-8"))
    return data[node]["ros__parameters"]


def test_e110_sdk_parameters_match_manual_and_slam_contract():
    params = load_parameters("lds_e110.yaml", "bluesea_node")
    assert params["type"] == "uart"
    assert params["baud_rate"] == 230400
    assert params["frame_id"] == "lidar_link"
    assert params["scan_topic"] == "/scan"
    assert params["output_scan"] is True
    assert params["output_360"] is True
    assert params["with_checksum"] is True
    assert params["min_dist"] == 0.05
    assert params["max_dist"] == 12.0


def test_slam_frames_ranges_and_mode_match_base_and_lidar():
    params = load_parameters("slam_toolbox.yaml", "slam_toolbox")
    assert params["mode"] == "mapping"
    assert params["map_frame"] == "map"
    assert params["odom_frame"] == "odom"
    assert params["base_frame"] == "base_footprint"
    assert params["scan_topic"] == "/scan"
    assert params["min_laser_range"] == 0.05
    assert params["max_laser_range"] == 12.0
    assert params["transform_publish_period"] > 0.0
    assert params["do_loop_closing"] is True


def test_urdf_has_required_tf_links_and_joint_names():
    root = element_tree.parse(PACKAGE / "urdf" / "kiwi.urdf.xacro").getroot()
    links = {item.attrib["name"] for item in root.findall("link")}
    joints = {item.attrib["name"] for item in root.findall("joint")}
    arguments = {
        item.attrib["name"]: item.attrib["default"]
        for item in root
        if item.tag.endswith("arg")
    }
    assert {
        "base_footprint",
        "base_link",
        "chassis_imu_link",
        "lidar_link",
        "livox_frame",
    } <= links
    assert {"wheel_joint_1", "wheel_joint_2", "wheel_joint_3"} <= joints
    assert "base_footprint_to_base_link" in joints
    assert "base_link_to_chassis_imu" in joints
    assert "base_link_to_lidar" in joints
    assert "lidar_link_to_livox_frame" in joints
    assert set(arguments) == {"hardware_config"}
    text = (PACKAGE / "urdf" / "kiwi.urdf.xacro").read_text(encoding="utf-8")
    assert 'name="chassis_imu_xyz" value="-0.0115 0.149 0.0"' in text
    assert 'name="chassis_imu_rpy" value="0.0 0.0 3.141592653589793"' in text
    assert 'name="lidar_xyz" value="0.0 0.0 0.20"' in text
    assert 'name="lidar_rpy" value="0.0 0.0 3.141592653589793"' in text
    assert '<origin xyz="0 0 ${model[\'base_z_m\']}" rpy="0 0 0"/>' in text
    assert 'xyz="${0.5 * wheel_center} ${-sin60 * wheel_center} 0"' in text
    assert 'xyz="${-wheel_center} 0 0" rpy="0 0 3.14159265359"' in text
    assert 'xyz="${0.5 * wheel_center} ${sin60 * wheel_center} 0"' in text
    assert "wheel_radius_m" in text
    assert "wheel_center_distance_m" in text


def test_launch_uses_yaml_and_does_not_duplicate_sensor_extrinsics():
    launch = (PACKAGE / "launch" / "kiwi_slam.launch.py").read_text(encoding="utf-8")
    for name in ("start_base", "start_ekf", "start_lidar", "start_slam", "start_monitor"):
        assert f'"{name}"' in launch
    for name in ("lidar_x", "lidar_y", "lidar_z", "lidar_yaw", "imu_x", "imu_y"):
        assert f'DeclareLaunchArgument("{name}"' not in launch
    assert '"robot_params_file"' in launch
    assert '"kiwi_robot.launch.py"' in launch
    assert '"rviz_software_rendering"' in launch
    assert 'default_value="true"' in launch
    assert 'FindPackageShare("bluesea2")' in launch
    assert 'executable="sync_slam_toolbox_node"' in launch
    assert '"rviz_config_file"' in launch
    assert '"kiwi_slam.rviz"' in launch
    active_lines = [
        line for line in launch.splitlines() if not line.lstrip().startswith("#")
    ]
    active_launch = "\n".join(active_lines)
    assert "kiwi_base_ekf.launch.py" not in active_launch
    assert "static_transform_publisher" not in active_launch
    assert 'name="rviz2"' not in active_launch
    assert '"QT_QPA_PLATFORM": "xcb"' in active_launch
    assert '"LIBGL_ALWAYS_SOFTWARE"' in active_launch

    robot_launch = (PACKAGE / "launch" / "kiwi_robot.launch.py").read_text(
        encoding="utf-8"
    )
    assert '" hardware_config:=", base_params' in robot_launch
    assert 'executable="kiwi_base_node"' in robot_launch
    assert 'executable="ekf_node"' in robot_launch
    assert 'executable="robot_state_publisher"' in robot_launch


def test_mid360s_network_and_fast_lio_contract():
    """Mid-360S 使用工控机有线地址、自定义点云和内置 IMU。"""
    network = json.loads(
        (PACKAGE / "config" / "mid360s_network.json").read_text(encoding="utf-8")
    )
    host = network["Mid360s"]["host_net_info"][0]
    lidar = network["lidar_configs"][0]
    extrinsic = lidar["extrinsic_parameter"]
    assert host["host_ip"] == "192.168.1.50"
    assert lidar["ip"] == "192.168.1.117"
    assert all(isinstance(extrinsic[key], int) for key in ("x", "y", "z"))
    assert len(
        {
            host["cmd_data_port"],
            host["push_msg_port"],
            host["point_data_port"],
            host["imu_data_port"],
            host["log_data_port"],
        }
    ) == 5

    params = load_parameters("fast_lio_mid360s.yaml", "/**")
    assert params["common"]["lid_topic"] == "/livox/lidar"
    assert params["common"]["imu_topic"] == "/livox/imu"
    assert params["preprocess"]["lidar_type"] == 1
    assert params["mapping"]["extrinsic_est_en"] is False
    assert params["publish"]["map_en"] is True


def test_lio_rviz_prefers_hardware_rendering():
    """工控机有 Intel 核显时，三维 RViz 默认不强制软件渲染。"""
    launch = (PACKAGE / "launch" / "kiwi_lio.launch.py").read_text(encoding="utf-8")
    assert '"rviz_software_rendering", default_value="false"' in launch

    driver = load_parameters("livox_mid360s.yaml", "livox_lidar_publisher")
    assert driver["xfer_format"] == 1
    assert driver["frame_id"] == "livox_frame"
    imu = load_parameters("mid360_imu.yaml", "mid360_imu_adapter")
    assert imu["input_topic"] == "/livox/imu"
    assert imu["output_topic"] == "/livox/imu/ekf"
    assert all(value > 0.0 for value in imu["angular_velocity_variance"])

    # Mid-360S 主路线只在独立 EKF YAML 中选择 Livox IMU 支路。
    ekf = load_parameters("ekf_mid360.yaml", "ekf_filter_node")
    assert ekf["odom0"] == "/chassis/odom/raw"
    assert ekf["imu0"] == "/livox/imu/ekf"
    assert [i for i, enabled in enumerate(ekf["imu0_config"]) if enabled] == [11]


def test_lio_launch_keeps_wheel_odom_and_lio_tf_ownership_separate():
    """LIO 启动必须复用轮式 odom，并隔离 FAST-LIO2 原生 body TF。"""
    launch = (PACKAGE / "launch" / "kiwi_lio.launch.py").read_text(encoding="utf-8")
    assert '"xfer_format": 1' not in launch
    assert '"frame_id": "livox_frame"' not in launch
    assert '"livox_mid360s.yaml"' in launch
    assert '"mid360_imu.yaml"' in launch
    assert 'executable="kiwi_mid360_imu_adapter"' in launch
    assert 'package="fast_lio"' in launch
    assert '("/tf", "/fast_lio/tf")' in launch
    assert '"kiwi_robot.launch.py"' in launch
    assert '"ekf_mid360.yaml"' in launch
    assert '"kiwi_slam.launch.py"' not in launch


def test_rviz_and_monitor_expose_map_robot_and_debug_contracts():
    rviz = yaml.safe_load((PACKAGE / "config" / "kiwi_slam.rviz").read_text(encoding="utf-8"))
    displays = rviz["Visualization Manager"]["Displays"]
    classes = {display["Class"] for display in displays}
    assert "rviz_default_plugins/Map" in classes
    assert "rviz_default_plugins/RobotModel" in classes
    assert "rviz_default_plugins/LaserScan" in classes
    assert rviz["Visualization Manager"]["Global Options"]["Fixed Frame"] == "odom"

    monitor = load_parameters("monitor.yaml", "kiwi_slam_monitor")
    assert monitor["map_topic"] == "/map"
    assert monitor["require_map_tf"] is True
