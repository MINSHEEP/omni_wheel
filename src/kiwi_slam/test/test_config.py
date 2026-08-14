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
    # 与 E110 时代 ±135° 前向视场一致，实机校准后允许修改此断言。
    assert params["angle_disable_min"] == [13500]
    assert params["angle_disable_max"] == [22500]


def test_m10p_range_and_scan_masking_match_consumers():
    """驱动量程必须覆盖 SLAM 与 costmap 的读数范围，避免静默丢数据。"""
    driver = load_parameters("ls_m10p_uart.yaml", "lslidar_driver_node")
    slam = load_parameters("slam_toolbox.yaml", "slam_toolbox")
    assert driver["max_range"] == slam["max_laser_range"]
    assert driver["min_range"] < 1.0
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
    assert {"base_footprint", "base_link", "chassis_imu_link", "lidar_link"} <= links
    assert {"wheel_joint_1", "wheel_joint_2", "wheel_joint_3"} <= joints
    assert "base_footprint_to_base_link" in joints
    assert "base_link_to_chassis_imu" in joints
    assert "base_link_to_lidar" in joints
    assert set(arguments) == {"hardware_config"}
    text = (PACKAGE / "urdf" / "kiwi.urdf.xacro").read_text(encoding="utf-8")
    assert 'name="chassis_imu_xyz" value="0.0115 -0.149 0.0"' in text
    # 与 kiwi.urdf.xacro 当前标定值保持一致；换雷达不改外参，待实机复测。
    assert 'name="lidar_xyz" value="0.32 0.0 0.20"' in text
    assert "wheel_radius_m" in text
    assert "wheel_center_distance_m" in text


def test_launch_uses_one_base_yaml_for_base_and_ekf():
    launch = (PACKAGE / "launch" / "kiwi_slam.launch.py").read_text(encoding="utf-8")
    for name in ("start_base", "start_ekf", "start_lidar", "start_slam", "start_monitor"):
        assert f'"{name}"' in launch
    # 雷达只允许经 kiwi_slam 的 M10P 封装启动；bluesea2 仅留在源码树供回退，
    # 断言针对 FindPackageShare 调用而非注释文字。
    assert '"m10p_uart_lidar.launch.py"' in launch
    assert 'FindPackageShare("bluesea2")' not in launch
    assert 'executable="sync_slam_toolbox_node"' in launch
    assert '"kiwi_robot.launch.py"' in launch
    assert "ekf_params_file" not in launch
    assert "kiwi_lio.launch.py" not in launch

    lidar_launch = (PACKAGE / "launch" / "m10p_uart_lidar.launch.py").read_text(
        encoding="utf-8"
    )
    # 驱动可执行文件创建普通 rclcpp::Node，封装内禁止引入生命周期管理；
    # 断言针对实际的动作调用与 import，不匹配注释文字。
    assert 'executable="lslidar_driver_node"' in lidar_launch
    assert 'name="lslidar_driver_node"' in lidar_launch
    assert "LifecycleNode(" not in lidar_launch
    assert "import LifecycleNode" not in lidar_launch
    assert '"ls_m10p_uart.yaml"' in lidar_launch

    robot_launch = (PACKAGE / "launch" / "kiwi_robot.launch.py").read_text(
        encoding="utf-8"
    )
    assert '" hardware_config:=", base_params' in robot_launch
    assert 'executable="kiwi_base_node"' in robot_launch
    assert 'executable="ekf_node"' in robot_launch
    assert 'parameters=[base_params, {"robot_description": robot_description}]' in robot_launch
    assert robot_launch.count("parameters=[base_params]") == 2

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
