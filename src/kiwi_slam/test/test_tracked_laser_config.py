"""双履带纯激光入口的跨包话题、TF 与启动契约。"""

from pathlib import Path

import pytest
import yaml

from kiwi_slam.runtime_contract import (
    RuntimeContractError,
    validate_tracked_laser_parameters,
)


WORKSPACE_SRC = Path(__file__).resolve().parents[2]
BASE = WORKSPACE_SRC / "kiwi_base/config/tracked_laser.yaml"
SLAM = WORKSPACE_SRC / "kiwi_slam"
LIDAR = SLAM / "config/ls_m10p_uart.yaml"
RF2O = SLAM / "config/tracked_rf2o.yaml"
GATE = SLAM / "config/tracked_mapping_gate.yaml"
SLAM_CONFIG = SLAM / "config/slam_toolbox.yaml"


def test_tracked_laser_contract_uses_only_rf2o_for_odom():
    validate_tracked_laser_parameters(BASE, LIDAR, RF2O, GATE, SLAM_CONFIG)
    rf2o = yaml.safe_load(RF2O.read_text(encoding="utf-8"))[
        "rf2o_laser_odometry"
    ]["ros__parameters"]
    assert rf2o["publish_tf"] is True
    assert rf2o["odom_topic"] == "/odometry/filtered"
    assert rf2o["init_pose_from_topic"] == ""
    launch = (SLAM / "launch/tracked_laser_mapping.launch.py").read_text(encoding="utf-8")
    assert 'executable="tracked_base_node"' in launch
    assert 'executable="rf2o_laser_odometry_node"' in launch
    assert '"m10p_uart_lidar.launch.py"' in launch
    assert 'executable="ekf_node"' not in launch
    assert 'executable="kiwi_base_node"' not in launch


def test_duplicate_or_mismatched_odom_tf_is_rejected(tmp_path):
    config = yaml.safe_load(RF2O.read_text(encoding="utf-8"))
    config["rf2o_laser_odometry"]["ros__parameters"]["publish_tf"] = False
    invalid = tmp_path / "rf2o.yaml"
    invalid.write_text(yaml.safe_dump(config), encoding="utf-8")
    with pytest.raises(RuntimeContractError, match="odom TF"):
        validate_tracked_laser_parameters(BASE, LIDAR, invalid, GATE, SLAM_CONFIG)


def test_imu_gate_cannot_be_enabled_in_laser_only_profile(tmp_path):
    config = yaml.safe_load(GATE.read_text(encoding="utf-8"))
    config["kiwi_mapping_scan_gate"]["ros__parameters"]["require_imu_ready"] = True
    invalid = tmp_path / "gate.yaml"
    invalid.write_text(yaml.safe_dump(config), encoding="utf-8")
    with pytest.raises(RuntimeContractError, match="IMU"):
        validate_tracked_laser_parameters(BASE, LIDAR, RF2O, invalid, SLAM_CONFIG)
