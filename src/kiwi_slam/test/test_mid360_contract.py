"""MID360 安装外参与 LIO/EKF 的安全接线契约。"""

import json
from pathlib import Path

import pytest
import yaml

from kiwi_slam.mid360_contract import validate_mount
from kiwi_slam.mid360_network import driver_config, validate_network


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config"
BASE = ROOT.parent / "kiwi_base/config/tracked_pwm.yaml"


def test_mount_matches_existing_lidar_pose_and_livox_internal_extrinsic():
    mount = validate_mount(CONFIG / "mid360_mount.yaml", CONFIG / "mid360_fastlio.yaml")
    model = yaml.safe_load(BASE.read_text())["tracked_robot_model"]["ros__parameters"]
    lidar_xyz = model["lidar_xyz"]
    old_footprint_lidar = [lidar_xyz[0], lidar_xyz[1], lidar_xyz[2] + model["base_z_m"]]
    derived_imu = [a - b for a, b in zip(old_footprint_lidar, mount["body_to_lidar_xyz"])]
    assert mount["base_to_body_xyz"] == pytest.approx(derived_imu)
    assert mount["base_to_body_rpy"] == model["lidar_rpy"] == [0.0, 0.0, 0.0]


def test_calibration_gate_and_lio_uses_internal_imu(tmp_path):
    mount_file = CONFIG / "mid360_mount.yaml"
    fastlio_file = CONFIG / "mid360_fastlio.yaml"
    data = yaml.safe_load(mount_file.read_text())
    data["mid360_mount"]["ros__parameters"]["calibrated"] = False
    uncalibrated = tmp_path / "uncalibrated.yaml"
    uncalibrated.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="未标定"):
        validate_mount(uncalibrated, fastlio_file)
    fastlio = yaml.safe_load(fastlio_file.read_text())["/**"]["ros__parameters"]
    assert fastlio["common"]["imu_topic"] == "/livox/imu"


def test_driver_config_routes_all_streams_to_chosen_host(monkeypatch):
    class Result:
        returncode = 0
        stderr = ""

        def __init__(self, stdout):
            self.stdout = stdout

    def fake_run(args, **_kwargs):
        if args[:3] == ["ip", "-j", "address"]:
            return Result(json.dumps([{"addr_info": [{"family": "inet", "local": "192.168.1.5"}]}]))
        return Result("192.168.1.12 dev enp2s0 src 192.168.1.5")

    monkeypatch.setattr("kiwi_slam.mid360_network.subprocess.run", fake_run)
    host, lidar = validate_network("enp2s0", "192.168.1.5", "192.168.1.12")
    config = driver_config(host, lidar)
    assert config["lidar_configs"][0]["ip"] == lidar
    info = config["Mid360s"]["host_net_info"]
    assert len(info) == 1
    assert info[0]["lidar_ip"] == [lidar]
    assert info[0]["host_ip"] == host
    assert [info[0][key] for key in
            ("cmd_data_port", "push_msg_port", "point_data_port", "imu_data_port")] == [
                56101, 56201, 56301, 56401]
    assert "MID360" in driver_config(host, lidar, "MID360")
    with pytest.raises(ValueError, match="型号"):
        driver_config(host, lidar, "other")
