"""Configuration ownership, relocated workspaces, and launch argument routing."""

from argparse import Namespace
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.utilities import perform_substitutions

from kiwi_web.config import PARAMETER_FILES, resolve_paths, workspace_root
from kiwi_web.server import load_config


def test_workspace_paths_follow_installation_not_working_directory(tmp_path, monkeypatch):
    for prefix in (tmp_path / "install", tmp_path / "install/kiwi_web"):
        assert workspace_root(prefix=prefix) == tmp_path
    monkeypatch.chdir("/tmp")
    import kiwi_web.config as config

    monkeypatch.setattr(config, "get_package_prefix", lambda _: str(tmp_path / "install/kiwi_web"))
    assert resolve_paths({"maps_dir": "maps"})["maps_dir"] == str(tmp_path / "maps")
    assert resolve_paths({"maps_dir": "data/maps"})["maps_dir"] == str(tmp_path / "data/maps")
    with pytest.raises(ValueError, match="非标准"):
        workspace_root(prefix="/opt/robot")
    assert workspace_root(str(tmp_path), prefix="/opt/robot") == tmp_path


def test_all_file_overrides_are_resolved_before_child_processes(tmp_path):
    config = dict(workspace=str(tmp_path), maps_dir="maps")
    for key in PARAMETER_FILES:
        path = tmp_path / "src" / key / "config.yaml"
        path.parent.mkdir(parents=True)
        path.write_text("{}")
        config[key] = str(path.relative_to(tmp_path))
    resolve_paths(config)
    assert all(Path(config[key]).is_absolute() for key in PARAMETER_FILES)
    config["slam_params_file"] = "missing.yaml"
    with pytest.raises(ValueError, match="slam_params_file"):
        resolve_paths(config)


def test_cli_base_override_controls_web_limits_and_ros_file(tmp_path):
    base = tmp_path / "base.yaml"
    from kiwi_platform import load_profile
    data = load_profile().data("base")
    data["kiwi_base_node"]["ros__parameters"].update(
        max_linear_speed_m_s=0.12, max_lateral_speed_m_s=0.14,
        max_angular_speed_rad_s=0.2
    )
    data["kiwi_teleop_keyboard"]["ros__parameters"].update(speed_m_s=0.1, yaw_rad_s=0.2)
    base.write_text(yaml.safe_dump(data))
    config, manual, navigation, _ = load_config(
        Namespace(config=None, workspace=str(tmp_path), base_params_file="base.yaml", port=8899)
    )
    assert tuple(manual) == navigation == (0.12, 0.14, 0.2)
    assert config["base_params_file"] == str(base)
    assert config["port"] == 8899
    assert config["maps_dir"] == str(tmp_path / "maps")


def test_web_launch_passes_all_explicit_overrides_to_server():
    spec = importlib.util.spec_from_file_location(
        "web_launch", Path(__file__).resolve().parents[1] / "launch/web.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations["config"] = "/package/config/web.yaml"
    for key in module.OVERRIDES:
        context.launch_configurations[key] = "/new/workspace/" + key if key != "port" else "8899"
    node = module.setup(context)[0]
    command = [perform_substitutions(context, word) for word in node.cmd[1:]]
    for key in module.OVERRIDES:
        flag = "--" + key.replace("_", "-")
        assert command[command.index(flag) + 1] == context.launch_configurations[key]


def test_web_limits_follow_tracked_profile_and_disable_lateral(tmp_path):
    selector = tmp_path / "active.yaml"
    selector.write_text("chassis: tracked_pwm\n")
    config, manual, navigation, _ = load_config(
        Namespace(config=None, workspace=str(tmp_path), platform_file=str(selector), port=8080)
    )
    assert config["active_chassis"] == "tracked_pwm"
    assert config["platform_file"] == str(selector)
    assert config["feedback_topic"] == "/chassis/odom/laser"
    assert config["imu_topic"] == "/chassis/imu/data_raw"
    assert config["imu_frame_id"] == "chassis_imu_link"
    assert config["motion_enabled"] is False
    assert manual[1] == navigation[1] == 0.0
    assert manual[0] > 0 and manual[2] > 0


def test_web_navigation_passes_selected_base_file_to_nav_core(tmp_path):
    import kiwi_web.config as web_config
    from kiwi_platform import load_profile

    selector = tmp_path / "active.yaml"
    selector.write_text("chassis: tracked_pwm\n")
    profile = load_profile(str(selector))
    base = profile.data("base")
    motor = base["tracked_base_node"]["ros__parameters"]
    motor.update(mapping_sensors_verified=True, hardware_calibrated=True,
                 enable_motion=True, open_loop_test_enabled=False, command_per_m_s=10.0)
    calibrated = tmp_path / "calibrated.yaml"
    calibrated.write_text(yaml.safe_dump(base))
    lidar = tmp_path / "lidar.yaml"
    lidar.write_text(yaml.safe_dump(profile.data("lidar")))
    map_file = tmp_path / "map.yaml"
    map_file.write_text("image: map.pgm\nresolution: 0.05\norigin: [0,0,0]\n")
    (tmp_path / "map.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    spec = importlib.util.spec_from_file_location(
        "web_stack", Path(__file__).resolve().parents[1] / "launch/stack.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(
        mode="navigation", map=str(map_file), platform_file=str(selector),
        expected_chassis="tracked_pwm",
        **{key: str(calibrated) if key == "base_params_file" else
           str(lidar) if key == "lidar_params_file" else ""
           for key in web_config.PARAMETER_FILES}
    )
    actions = module.setup(context)
    assert len(actions) == 2
    robot_include = next(action for action in actions[0].get_sub_entities()
                         if action.__class__.__name__ == "IncludeLaunchDescription")
    robot_arguments = dict(robot_include._IncludeLaunchDescription__launch_arguments)
    assert robot_arguments["base_params_file"] == str(calibrated)
    assert robot_arguments["lidar_params_file"] == str(lidar)
    navigation_group = actions[1]
    include = next(action for action in navigation_group.get_sub_entities()
                   if action.__class__.__name__ == "IncludeLaunchDescription")
    arguments = dict(include._IncludeLaunchDescription__launch_arguments)
    assert arguments["base_params_file"] == str(calibrated)
    assert arguments["lidar_params_file"] == str(lidar)
    assert arguments["platform_file"] == str(selector)
    context.launch_configurations["expected_chassis"] = "kiwi"
    with pytest.raises(ValueError, match="已变化"):
        module.setup(context)
