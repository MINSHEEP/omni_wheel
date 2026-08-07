"""Contract tests for E110, SLAM, launch, and URDF integration files."""

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
    assert {"base_footprint", "base_link", "imu_link", "lidar_link"} <= links
    assert {"wheel_joint_1", "wheel_joint_2", "wheel_joint_3"} <= joints
    assert "base_footprint_to_base_link" in joints
    assert "base_link_to_lidar" in joints
    assert arguments["imu_x"] == "0.0115"
    assert arguments["imu_y"] == "-0.149"


def test_launch_exposes_hardware_free_and_mounting_debug_arguments():
    launch = (PACKAGE / "launch" / "kiwi_slam.launch.py").read_text(encoding="utf-8")
    for name in ("start_base", "start_lidar", "start_slam", "start_monitor"):
        assert f'DeclareLaunchArgument("{name}"' in launch
    for name in ("lidar_x", "lidar_y", "lidar_z", "lidar_yaw"):
        assert f'DeclareLaunchArgument("{name}"' in launch
    assert 'DeclareLaunchArgument("imu_x", default_value="0.0115")' in launch
    assert 'DeclareLaunchArgument("imu_y", default_value="-0.149")' in launch
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
