"""MID360 投影扫描与履带 Nav2 的跨参数契约。"""

from copy import deepcopy
from pathlib import Path

import yaml


CONFIG = Path(__file__).resolve().parents[1] / "config"


def _load(name):
    return yaml.safe_load((CONFIG / name).read_text(encoding="utf-8"))


def _costmap(config, scope):
    return config[scope][scope]["ros__parameters"]


def test_mid360_amcl_and_costmaps_consume_the_same_projected_scan():
    config = _load("mid360_tracked_nav2.yaml")
    amcl = config["amcl"]["ros__parameters"]
    assert amcl["robot_model_type"] == "nav2_amcl::DifferentialMotionModel"
    assert amcl["scan_topic"] == "/scan_mid360"
    assert (amcl["laser_min_range"], amcl["laser_max_range"]) == (0.5, 20.0)
    assert (amcl["global_frame_id"], amcl["odom_frame_id"], amcl["base_frame_id"]) == (
        "map", "odom", "base_footprint"
    )
    assert amcl["tf_broadcast"] is True
    for node in ("bt_navigator", "controller_server", "velocity_smoother"):
        assert config[node]["ros__parameters"]["odom_topic"] == "/fastlio/odom_base"

    scans = []
    for scope, frame in (("local_costmap", "odom"), ("global_costmap", "map")):
        costmap = _costmap(config, scope)
        assert costmap["global_frame"] == frame
        assert costmap["robot_base_frame"] == "base_footprint"
        assert costmap["obstacle_layer"]["observation_sources"] == "scan"
        scan = costmap["obstacle_layer"]["scan"]
        assert scan["topic"] == amcl["scan_topic"]
        assert scan["data_type"] == "LaserScan"
        assert scan["inf_is_valid"] is False
        assert scan["obstacle_min_range"] == scan["raytrace_min_range"] == amcl["laser_min_range"]
        assert scan["obstacle_max_range"] == 8.0
        assert scan["raytrace_max_range"] == 10.0
        assert scan["raytrace_max_range"] <= amcl["laser_max_range"]
        scans.append(scan)
    assert scans[0] == scans[1]


def test_mid360_preserves_tracked_motion_and_planning_parameters():
    original = _load("tracked_nav2.yaml")
    expected = deepcopy(original)
    expected["amcl"]["ros__parameters"].update(
        scan_topic="/scan_mid360", laser_min_range=0.5, laser_max_range=20.0
    )
    for node in ("bt_navigator", "controller_server", "velocity_smoother"):
        expected[node]["ros__parameters"]["odom_topic"] = "/fastlio/odom_base"
    for scope in ("local_costmap", "global_costmap"):
        scan = _costmap(expected, scope)["obstacle_layer"]["scan"]
        scan.update(
            topic="/scan_mid360", inf_is_valid=False,
            obstacle_min_range=0.5, raytrace_min_range=0.5,
        )
    assert _load("mid360_tracked_nav2.yaml") == expected
