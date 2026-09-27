"""Mapping and localization select exactly one map-to-odom authority."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


LAUNCH = Path(__file__).resolve().parents[1] / "launch/mid360_navigation.launch.py"
SPEC = importlib.util.spec_from_file_location("mid360_navigation_launch", LAUNCH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _actions(mode, map_file=""):
    context = LaunchContext()
    context.launch_configurations.update({
        "mode": mode, "map": str(map_file), "platform_file": "",
        "mount_file": "", "fastlio_params_file": "",
        "nav2_params_file": "", "slam_params_file": "",
        "map_pcd": "/tmp/mid360_map.pcd", "start_base": "false",
        "start_driver": "false", "start_nav2": "false", "use_rviz": "false",
        "lidar_interface": "", "host_ip": "", "lidar_ip": "",
        "lidar_model": "Mid360s",
        "scan_z_min": "0.10", "scan_z_max": "1.20",
    })
    return context, MODULE._setup(context)


def test_mapping_uses_slam_without_amcl():
    context, actions = _actions("mapping")
    assert sum(isinstance(action, Node) and action.node_package == "slam_toolbox"
               for action in actions) == 1
    included = [perform_substitutions(context, action.launch_description_source._LaunchDescriptionSource__location)
                for action in actions if isinstance(action, IncludeLaunchDescription)]
    assert any(path.endswith("mid360_pipeline.launch.py") for path in included)
    assert not any(path.endswith("localization_launch.py") for path in included)
    pipeline = next(action for action in actions if isinstance(action, IncludeLaunchDescription))
    assert "ekf_params_file" not in dict(pipeline._IncludeLaunchDescription__launch_arguments)


def test_localization_uses_amcl_without_slam(tmp_path):
    image = tmp_path / "map.pgm"
    image.write_bytes(b"P5\n1 1\n255\n\xff")
    map_file = tmp_path / "map.yaml"
    map_file.write_text("image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n", encoding="utf-8")
    context, actions = _actions("localization", map_file)
    assert not any(isinstance(action, Node) and action.node_package == "slam_toolbox"
                   for action in actions)
    included = [perform_substitutions(context, action.launch_description_source._LaunchDescriptionSource__location)
                for action in actions if isinstance(action, IncludeLaunchDescription)]
    assert any(path.endswith("localization_launch.py") for path in included)
    assert not any(path.endswith("mapping_navigation.launch.py") for path in included)
    pipeline = next(action for action in actions if isinstance(action, IncludeLaunchDescription)
                    and "mid360_pipeline.launch.py" in perform_substitutions(
                        context, action.launch_description_source._LaunchDescriptionSource__location))
    assert "ekf_params_file" not in dict(pipeline._IncludeLaunchDescription__launch_arguments)
