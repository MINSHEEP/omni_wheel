"""Web 的私有速度入口不能与 MID360 安全门控输出形成旁路或回路。"""

import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


LAUNCH = Path(__file__).resolve().parents[1] / "launch/mid360_pipeline.launch.py"
SPEC = importlib.util.spec_from_file_location("mid360_pipeline_web_launch", LAUNCH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _context(topic, base_file=""):
    context = LaunchContext()
    context.launch_configurations.update({
        "platform_file": "", "base_params_file": base_file,
        "mount_file": "", "fastlio_params_file": "",
        "mode": "mapping", "map_pcd": "/tmp/mid360_web_test.pcd",
        "start_base": "true", "start_driver": "false",
        "motion_gate_input_topic": topic, "lidar_interface": "",
        "host_ip": "", "lidar_ip": "", "lidar_model": "Mid360s",
        "scan_z_min": "0.10", "scan_z_max": "1.20",
    })
    return context


def test_web_command_goes_through_mid360_gate_and_base_override(tmp_path, monkeypatch):
    original_share = MODULE.get_package_share_directory
    monkeypatch.setattr(MODULE, "get_package_share_directory",
                        lambda package: str(tmp_path) if package == "fast_lio"
                        else original_share(package))
    from kiwi_platform import load_profile

    profile = load_profile(required_keys=("base",))
    context = _context("/kiwi_web/cmd_vel", profile.path("base"))
    actions = MODULE._setup(context)
    gate = next(node for node in actions if isinstance(node, Node)
                and node.node_executable == "mid360_motion_gate")
    gate_params = {
        perform_substitutions(context, name): yaml.safe_load(perform_substitutions(context, value))
        for name, value in gate._Node__parameters[0].items()
    }
    assert gate_params["input_topic"] == "/kiwi_web/cmd_vel"
    base = next(node for node in actions if isinstance(node, Node)
                and node.node_executable == "tracked_base_node")
    assert Path(perform_substitutions(context, base._Node__parameters[0].param_file)).resolve() == Path(profile.path("base")).resolve()
    base_params = {
        perform_substitutions(context, name): yaml.safe_load(perform_substitutions(context, value))
        for name, value in base._Node__parameters[1].items()
    }
    assert base_params["cmd_vel_topic"] == "/mid360/cmd_vel_safe"


@pytest.mark.parametrize("topic", ["cmd_vel", "/mid360/cmd_vel_safe", ""])
def test_gate_rejects_relative_or_output_topic(topic):
    with pytest.raises(ValueError, match="速度门控输入"):
        MODULE._setup(_context(topic))
