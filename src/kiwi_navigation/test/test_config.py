"""Nav2 参数、启动模式和 RViz 坐标系静态检查."""

from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def load_yaml(name: str):
    """读取被测配置文件."""
    return yaml.safe_load((PACKAGE / "config" / name).read_text(encoding="utf-8"))


def test_nav2_uses_omni_frames_and_lateral_sampling():
    """DWB、AMCL 和 costmap 必须按三轮全向底盘配置."""
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


def test_speed_relay_matches_base_input():
    """Nav2 最终速度必须进入底盘现有的 /cmd_vel_safe."""
    relay = load_yaml("cmd_relay.yaml")["kiwi_nav_cmd_relay"]["ros__parameters"]
    assert relay["input_topic"] == "/cmd_vel"
    assert relay["output_topic"] == "/cmd_vel_safe"
    assert relay["command_timeout_s"] <= 0.30


def test_launch_modes_do_not_mix_slam_and_amcl():
    """建图模式启动 SLAM，已知地图模式关闭 SLAM 并启动 AMCL."""
    mapping = (PACKAGE / "launch" / "mapping_navigation.launch.py").read_text(
        encoding="utf-8"
    )
    localization = (PACKAGE / "launch" / "localization_navigation.launch.py").read_text(
        encoding="utf-8"
    )
    assert '"start_slam": start_slam' in mapping
    assert '"start_slam": "false"' in localization
    assert "localization_launch.py" in localization


def test_navigation_rviz_uses_map_frame():
    """Nav2 RViz 固定坐标系必须属于二维 map-odom-base TF 树."""
    rviz = (PACKAGE / "config" / "navigation.rviz").read_text(encoding="utf-8")
    assert "Fixed Frame: map" in rviz
    assert "camera_init" not in rviz
