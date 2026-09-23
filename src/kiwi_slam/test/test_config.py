"""检查镭神 M10P、SLAM、统一底盘参数和 URDF 接口。"""

from pathlib import Path
import xml.etree.ElementTree as element_tree

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def load_parameters(name, node):
    data = yaml.safe_load((PACKAGE / "config" / name).read_text(encoding="utf-8"))
    return data[node]["ros__parameters"]


def test_m10p_driver_parameters_match_slam_contract():
    """锁定 M10P 驱动的节点名、话题、frame 与量程，防止与消费方脱节。"""
    params = load_parameters("ls_m10p_uart.yaml", "lslidar_driver_node")
    assert params["lidar_type"] == "X10"
    assert params["lidar_model"] == "M10P"
    assert params["serial_port"] == "/dev/wheeltec_lidar"
    assert params["frame_id"] == "lidar_link"
    assert params["laserscan_topic"] == "/scan"
    assert params["publish_scan"] is True
    assert params["use_time_service"] is False
    assert params["min_range"] == 0.15
    assert params["max_range"] == 12.0
    # 裁剪扇区单位 0.01 度；当前值基于「0° 朝前」安装假设屏蔽后方 135°~225°，
    # 实机校准后允许修改此断言。
    assert params["angle_disable_min"] == [13500]
    assert params["angle_disable_max"] == [22500]


def test_m10p_range_and_scan_masking_match_consumers():
    """驱动量程必须覆盖 SLAM 与 costmap 的读数范围，避免静默丢数据。"""
    driver = load_parameters("ls_m10p_uart.yaml", "lslidar_driver_node")
    slam = load_parameters("slam_toolbox.yaml", "slam_toolbox")
    assert driver["max_range"] == slam["max_laser_range"]
    assert driver["min_range"] == slam["min_laser_range"]
    # 裁剪扇区必须成对且为 0.01 度整数。
    assert len(driver["angle_disable_min"]) == len(driver["angle_disable_max"])
    for low, high in zip(driver["angle_disable_min"], driver["angle_disable_max"]):
        assert isinstance(low, int) and isinstance(high, int)
        assert 0 <= low <= high <= 36000


def test_slam_frames_ranges_and_mode_match_base_and_lidar():
    params = load_parameters("slam_toolbox.yaml", "slam_toolbox")
    assert params["mode"] == "mapping"
    assert params["map_frame"] == "map"
    assert params["odom_frame"] == "odom"
    assert params["base_frame"] == "base_footprint"
    assert params["scan_topic"] == "/scan_slam"
    assert params["min_laser_range"] == 0.15
    assert params["max_laser_range"] == 12.0
    assert params["transform_publish_period"] > 0.0
    assert params["do_loop_closing"] is True
    # 限制关键帧密度，避免短基线节点持续占用 CPU。
    assert params["minimum_travel_distance"] == 0.20
    assert params["minimum_travel_heading"] == 0.20


def test_urdf_has_required_tf_links_and_joint_names():
    root = element_tree.parse(PACKAGE / "urdf" / "kiwi.urdf.xacro").getroot()
    links = {item.attrib["name"] for item in root.findall("link")}
    joints = {item.attrib["name"] for item in root.findall("joint")}
    arguments = {
        item.attrib["name"]: item.attrib["default"]
        for item in root
        if item.tag.endswith("arg")
    }
    assert {"base_footprint", "base_link", "chassis_imu_link", "lidar_link"} <= links
    assert {"wheel_joint_1", "wheel_joint_2", "wheel_joint_3"} <= joints
    assert "base_footprint_to_base_link" in joints
    assert "base_link_to_chassis_imu" in joints
    assert "base_link_to_lidar" in joints
    assert set(arguments) == {"hardware_config"}
    text = (PACKAGE / "urdf" / "kiwi.urdf.xacro").read_text(encoding="utf-8")
    # 锁定第二版底盘安装值（127417e）；实机复测后同步修改 URDF 与本断言。
    assert 'name="chassis_imu_xyz" value="-0.0118 0.0 0.0"' in text
    assert 'name="lidar_xyz" value="0.209 0.0 0.20"' in text
    assert "wheel_radius_m" in text
    assert "wheel_center_distance_m" in text


def test_launch_routes_base_ekf_lidar_gate_and_slam_through_profile():
    launch = (PACKAGE / "launch" / "kiwi_slam.launch.py").read_text(encoding="utf-8")
    shared = (PACKAGE.parent / "kiwi_platform" / "kiwi_platform" / "launching.py").read_text(encoding="utf-8")
    for name in ("start_base", "start_ekf", "start_lidar", "start_slam", "start_monitor"):
        assert f'"{name}"' in launch
    assert "load_profile(" in launch
    assert "robot_actions(profile" in launch
    assert "mapping_actions(profile" in launch
    for executable in ("lslidar_driver_node", "rf2o_laser_odometry_node", "kiwi_mapping_scan_gate",
                       "sync_slam_toolbox_node", "ekf_node"):
        assert executable in shared
    assert "static_transform_publisher" not in launch


def test_mapping_gate_preserves_raw_scan_and_requires_real_calibration():
    """SLAM 使用内部受控扫描，原始 /scan、Frame 与 EKF 话题保持既有契约。"""
    gate = load_parameters("mapping_gate.yaml", "kiwi_mapping_scan_gate")
    base = load_parameters(
        str(PACKAGE.parent / "kiwi_base" / "config" / "kiwi_base.yaml"),
        "kiwi_base_node",
    )
    slam = load_parameters("slam_toolbox.yaml", "slam_toolbox")
    assert gate["input_scan_topic"] == "/scan"
    assert gate["output_scan_topic"] == slam["scan_topic"] == "/scan_slam"
    assert gate["odom_topic"] == "/odometry/filtered"
    assert gate["imu_ready_topic"] == base["imu_calibration_ready_topic"]
    assert gate["laser_frame"] == "lidar_link"
    assert gate["odom_frame"] == "odom"
    assert gate["base_frame"] == "base_footprint"
    assert gate["required_stable_scans"] >= 2
    assert 0.0 < gate["min_valid_scan_ratio"] <= 1.0
    assert 0.0 < gate["min_scan_overlap_ratio"] <= 1.0


def test_rviz_and_monitor_expose_map_robot_and_debug_contracts():
    rviz = yaml.safe_load((PACKAGE / "config" / "kiwi_slam.rviz").read_text(encoding="utf-8"))
    displays = rviz["Visualization Manager"]["Displays"]
    classes = {display["Class"] for display in displays}
    assert "rviz_default_plugins/Map" in classes
    assert "rviz_default_plugins/RobotModel" in classes
    assert "rviz_default_plugins/LaserScan" in classes
    assert rviz["Visualization Manager"]["Global Options"]["Fixed Frame"] == "odom"

    robot_model = next(
        display
        for display in displays
        if display["Class"] == "rviz_default_plugins/RobotModel"
    )
    # robot_description 为 Transient Local 发布，RViz 需匹配耐久性
    # 才能在后启动时收到。
    assert robot_model["Description Topic"]["Durability Policy"] == "Transient Local"

    monitor = load_parameters("monitor.yaml", "kiwi_slam_monitor")
    assert monitor["map_topic"] == "/map"
    assert monitor["require_map_tf"] is True
