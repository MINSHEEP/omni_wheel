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
    base.write_text(
        yaml.safe_dump(
            {
                "kiwi_base_node": {
                    "ros__parameters": {
                        "max_linear_speed_m_s": 0.12,
                        "max_lateral_speed_m_s": 0.14,
                        "max_angular_speed_rad_s": 0.2,
                    }
                }
            }
        )
    )
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
