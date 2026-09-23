"""全局选择、跨参数文件契约以及履带的标定关锁。"""
from pathlib import Path

import pytest
import yaml

import kiwi_platform.profile as profiles
from kiwi_platform.launching import robot_actions

SOURCE = Path(__file__).resolve().parents[2]


@pytest.fixture
def source_shares(monkeypatch, tmp_path):
    monkeypatch.setattr(profiles, "get_package_share_directory", lambda package: str(SOURCE / package))
    monkeypatch.setattr(profiles, "get_package_prefix", lambda package: str(SOURCE.parent / "install" / package))
    for chassis in ("kiwi", "tracked_pwm"):
        (tmp_path / f"{chassis}.yaml").write_text(f"chassis: {chassis}\n")
    return tmp_path


def test_selection_changes_all_parameter_files_and_driver(source_shares, monkeypatch):
    import kiwi_platform.launching as launching
    monkeypatch.setattr(launching, "get_package_share_directory", lambda _: "/tmp")
    kiwi = profiles.load_profile(str(source_shares / "kiwi.yaml"))
    tracked = profiles.load_profile(str(source_shares / "tracked_pwm.yaml"))
    assert kiwi.driver == "kiwi_base_node"
    assert tracked.driver == "tracked_base_node"
    assert kiwi.odometry == "wheel" and tracked.odometry == "rf2o"
    assert set(kiwi.files) == set(tracked.files) - {"rf2o"}
    for key in ("base", "nav2", "slam", "gate", "urdf"):
        assert kiwi.path(key) != tracked.path(key)
    assert profiles.validate_profile(kiwi, "mapping")
    assert not profiles.validate_profile(tracked, "driver")
    with pytest.raises(ValueError, match="尚未核验"):
        profiles.validate_profile(tracked, "mapping")
    nodes = robot_actions(tracked)
    names = [action.node_executable for action in nodes]
    assert names == ["robot_state_publisher", "tracked_base_node", "lslidar_driver_node",
                     "rf2o_laser_odometry_node", "ekf_node"]
    nodes = robot_actions(kiwi)
    assert "rf2o_laser_odometry_node" not in [action.node_executable for action in nodes]


def test_tracked_mapping_waits_for_sensor_verification_but_not_speed(source_shares, tmp_path):
    original = profiles.load_profile(str(source_shares / "tracked_pwm.yaml"))
    base = original.data("base")
    motor = base["tracked_base_node"]["ros__parameters"]
    motor["mapping_sensors_verified"] = True
    assert not motor["hardware_calibrated"] and not motor["enable_motion"]
    override = tmp_path / "tracked_verified.yaml"
    override.write_text(yaml.safe_dump(base))
    tracked = profiles.load_profile(str(source_shares / "tracked_pwm.yaml"), {"base": str(override)})
    assert profiles.validate_profile(tracked, "mapping") is False
    with pytest.raises(ValueError, match="导航须关闭开环试车"):
        profiles.validate_profile(tracked, "localization")


def test_mixed_base_and_nav2_are_rejected(source_shares):
    tracked = profiles.load_profile(str(source_shares / "tracked_pwm.yaml"))
    kiwi = profiles.load_profile(str(source_shares / "kiwi.yaml"))
    wrong_base = profiles.load_profile(str(source_shares / "tracked_pwm.yaml"),
                                       {"base": kiwi.path("base")})
    with pytest.raises(ValueError, match="tracked_base_node"):
        profiles.validate_profile(wrong_base)
    base = tracked.data("base")
    motor = base["tracked_base_node"]["ros__parameters"]
    motor.update(mapping_sensors_verified=True, hardware_calibrated=True,
                 enable_motion=True, open_loop_test_enabled=False, command_per_m_s=10.0)
    calibrated = source_shares / "calibrated.yaml"
    calibrated.write_text(yaml.safe_dump(base))
    wrong_nav = profiles.load_profile(str(source_shares / "tracked_pwm.yaml"),
                                      {"base": str(calibrated), "nav2": kiwi.path("nav2")})
    with pytest.raises(ValueError, match="外形|差速模型"):
        profiles.validate_profile(wrong_nav, "localization")
    assert tracked.data("rf2o")["rf2o_laser_odometry"]["ros__parameters"]["init_pose_from_topic"] == ""


def test_invalid_selector_and_missing_files_fail_before_hardware(source_shares, tmp_path):
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text("chassis: mecanum\n")
    with pytest.raises(ValueError, match="chassis 无效"):
        profiles.load_profile(str(invalid))
    with pytest.raises(ValueError, match="文件不存在"):
        profiles.load_profile(str(source_shares / "kiwi.yaml"), {"nav2": str(tmp_path / "missing.yaml")})


def test_driver_and_mapping_do_not_require_navigation_files(source_shares, monkeypatch, tmp_path):
    share = profiles.get_package_share_directory
    prefix = profiles.get_package_prefix

    def without_navigation(package):
        if package == "kiwi_navigation":
            raise RuntimeError("导航包未安装")
        return share(package)

    monkeypatch.setattr(profiles, "get_package_share_directory", without_navigation)
    monkeypatch.setattr(profiles, "get_package_prefix", lambda package: (
        (_ for _ in ()).throw(RuntimeError("导航包未安装")) if package == "kiwi_navigation"
        else prefix(package)))
    kiwi = profiles.load_profile(str(source_shares / "kiwi.yaml"), required_keys=("base",))
    assert profiles.validate_profile(kiwi, "driver")
    mapping = profiles.load_profile(str(source_shares / "kiwi.yaml"),
                                    required_keys=("base", "lidar", "gate", "slam", "monitor"))
    assert profiles.validate_profile(mapping, "mapping")


def test_rf2o_missing_initialization_is_rejected_even_by_ekf_chain(source_shares, tmp_path):
    tracked = profiles.load_profile(str(source_shares / "tracked_pwm.yaml"))
    data = tracked.data("rf2o")
    data["rf2o_laser_odometry"]["ros__parameters"]["init_pose_from_topic"] = "/base_pose_ground_truth"
    wrong = tmp_path / "rf2o_wrong.yaml"
    wrong.write_text(yaml.safe_dump(data))
    profile = profiles.load_profile(str(source_shares / "tracked_pwm.yaml"), {"rf2o": str(wrong)})
    with pytest.raises(ValueError, match="原点初始化"):
        profiles.validate_rf2o(profile)


def test_lidar_ranges_and_tracked_lateral_limit_cannot_silently_diverge(source_shares, tmp_path):
    selector = str(source_shares / "tracked_pwm.yaml")
    original = profiles.load_profile(selector)
    base = original.data("base")
    base["tracked_base_node"]["ros__parameters"]["max_lateral_speed_m_s"] = 0.1
    wrong_base = tmp_path / "wrong_base.yaml"
    wrong_base.write_text(yaml.safe_dump(base))
    with pytest.raises(ValueError, match="履带横移上限"):
        profiles.validate_profile(profiles.load_profile(selector, {"base": str(wrong_base)}))
    lidar = original.data("lidar")
    lidar["lslidar_driver_node"]["ros__parameters"]["max_range"] = 10.0
    wrong_lidar = tmp_path / "wrong_lidar.yaml"
    wrong_lidar.write_text(yaml.safe_dump(lidar))
    base["tracked_base_node"]["ros__parameters"].pop("max_lateral_speed_m_s")
    base["tracked_base_node"]["ros__parameters"]["mapping_sensors_verified"] = True
    verified = tmp_path / "verified.yaml"
    verified.write_text(yaml.safe_dump(base))
    with pytest.raises(ValueError, match="SLAM 最大量程"):
        profiles.validate_profile(profiles.load_profile(selector,
            {"base": str(verified), "lidar": str(wrong_lidar)}), "mapping")
