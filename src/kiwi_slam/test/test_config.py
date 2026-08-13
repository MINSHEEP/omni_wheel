"""检查 E110、SLAM、统一底盘参数和 URDF 接口。"""

from pathlib import Path
import xml.etree.ElementTree as element_tree

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def load_parameters(name, node):
    data = yaml.safe_load((PACKAGE / "config" / name).read_text(encoding="utf-8"))
    return data[node]["ros__parameters"]


def test_e110_sdk_parameters_match_slam_contract():
    params = load_parameters("lds_e110.yaml", "bluesea_node")
    assert params["type"] == "uart"
    assert params["baud_rate"] == 230400
    assert params["frame_id"] == "lidar_link"
    assert params["scan_topic"] == "/scan"
    assert params["output_scan"] is True
    assert params["output_360"] is True
    assert params["scan_points"] == 400
    assert params["min_range"] == 0.05
    assert params["max_range"] == 12.0
    assert params["with_checksum"] is True
    assert params["min_dist"] == 0.05
    assert params["max_dist"] == 12.0


def test_laser_scan_uses_true_angles_and_fixed_grid():
    """防止 E110 的 398/399/400 点帧重新按点数均分而造成整圈跳变。"""
    source = (PACKAGE.parent / "bluesea-ros2" / "src" / "node.cpp").read_text(encoding="utf-8")
    assert "std::llround" in source
    assert "argdata.scan_points > 0" in source
    assert "for (const auto &point : hub->consume)" in source


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
    assert 'name="lidar_xyz" value="0.0 0.0 0.20"' in text
    assert "wheel_radius_m" in text
    assert "wheel_center_distance_m" in text


def test_launch_uses_one_base_yaml_for_base_and_ekf():
    launch = (PACKAGE / "launch" / "kiwi_slam.launch.py").read_text(encoding="utf-8")
    for name in ("start_base", "start_ekf", "start_lidar", "start_slam", "start_monitor"):
        assert f'"{name}"' in launch
    assert 'FindPackageShare("bluesea2")' in launch
    assert 'executable="sync_slam_toolbox_node"' in launch
    assert '"kiwi_robot.launch.py"' in launch
    assert "ekf_params_file" not in launch
    assert "kiwi_lio.launch.py" not in launch

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
