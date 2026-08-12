"""检查 E110 Nav2 参数和启动模式。"""

from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def load_yaml(name: str):
    return yaml.safe_load((PACKAGE / "config" / name).read_text(encoding="utf-8"))


def test_nav2_uses_omni_frames_and_lateral_sampling():
    config = load_yaml("nav2.yaml")
    amcl = config["amcl"]["ros__parameters"]
    controller = config["controller_server"]["ros__parameters"]
    dwb = controller["FollowPath"]
    assert amcl["robot_model_type"] == "nav2_amcl::OmniMotionModel"
    assert amcl["base_frame_id"] == "base_footprint"
    assert controller["odom_topic"] == "/odometry/filtered"
    assert dwb["min_vel_y"] < 0.0 < dwb["max_vel_y"]
    assert dwb["vy_samples"] > 1
    assert config["global_costmap"]["global_costmap"]["ros__parameters"]["global_frame"] == "map"
    assert config["local_costmap"]["local_costmap"]["ros__parameters"]["global_frame"] == "odom"


def test_velocity_smoother_is_the_direct_cmd_vel_output():
    config = load_yaml("nav2.yaml")
    smoother = config["velocity_smoother"]["ros__parameters"]
    assert smoother["max_velocity"] == [0.30, 0.30, 0.80]
    assert smoother["velocity_timeout"] == 0.50
    assert not (PACKAGE / "config" / "cmd_relay.yaml").exists()


def test_launch_modes_use_e110_without_adapter_or_relay():
    mapping = (PACKAGE / "launch" / "mapping_navigation.launch.py").read_text(
        encoding="utf-8"
    )
    localization = (PACKAGE / "launch" / "localization_navigation.launch.py").read_text(
        encoding="utf-8"
    )
    assert "condition=IfCondition(start_slam)" in mapping
    assert "slam_toolbox" not in localization
    assert "localization_launch.py" in localization
    for launch in (mapping, localization):
        assert 'FindPackageShare("bluesea2")' in launch
        assert "lidar_type" not in launch
        assert "radar_source.launch.py" not in launch
        assert "kiwi_nav_cmd_relay" not in launch


def test_navigation_rviz_uses_map_frame():
    rviz = (PACKAGE / "config" / "navigation.rviz").read_text(encoding="utf-8")
    assert "Fixed Frame: map" in rviz
    assert "camera_init" not in rviz
