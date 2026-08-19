"""统一底盘参数和 colcon overlay 防混用契约测试。"""

from pathlib import Path

import pytest
import yaml

from kiwi_slam.runtime_contract import (
    RuntimeContractError,
    validate_base_parameters,
    validate_workspace_overlay,
    workspace_install_root,
)


PACKAGE = Path(__file__).resolve().parents[1]


def test_current_base_yaml_satisfies_runtime_contract():
    parameter_file = PACKAGE.parent / "kiwi_base" / "config" / "kiwi_base.yaml"
    assert validate_base_parameters(parameter_file) == parameter_file.resolve()


def test_old_base_only_yaml_is_rejected_before_nodes_start(tmp_path):
    """复现事故根因：旧 YAML 没有 ekf_filter_node 时必须直接阻断。"""
    old_style = {
        "kiwi_base_node": {
            "ros__parameters": {
                "publish_rate": 50.0,
                "feedback_framed": True,
                "cmd_vel_timeout_s": 0.5,
                "wheel_odom_topic": "/chassis/odom/raw",
                "imu_topic": "/chassis/imu/data_raw",
                "odom_frame_id": "odom",
                "base_frame_id": "base_footprint",
            }
        }
    }
    parameter_file = tmp_path / "kiwi_base.yaml"
    parameter_file.write_text(yaml.safe_dump(old_style), encoding="utf-8")

    with pytest.raises(RuntimeContractError, match="ekf_filter_node"):
        validate_base_parameters(parameter_file)


def test_workspace_install_root_supports_isolated_and_merged_layouts():
    assert workspace_install_root(
        "/opt/kiwi/install/kiwi_base", "kiwi_base"
    ) == Path("/opt/kiwi/install")
    assert workspace_install_root(
        "/opt/kiwi/install", "kiwi_base"
    ) == Path("/opt/kiwi/install")


def test_active_workspace_packages_are_colocated():
    """测试环境也不得从其他 overlay 偷取同名 kiwi_base。"""
    root = validate_workspace_overlay(required_packages=("kiwi_base",))
    assert root.name == "install"
