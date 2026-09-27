"""Fail closed on incomplete MID360 calibration and LIO topic contracts."""

import math
from pathlib import Path

import yaml


def _parameters(path, node):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    try:
        return data[node]["ros__parameters"]
    except (TypeError, KeyError) as error:
        raise ValueError(f"{path} 缺少 {node}.ros__parameters") from error


def validate_mount(path, fastlio_path):
    mount = _parameters(path, "mid360_mount")
    if mount.get("calibrated") is not True:
        raise ValueError(f"MID360 外参未标定：请实测 {path} 的 base_to_body 并将 calibrated 设为 true")
    for key in ("base_to_body_xyz", "base_to_body_rpy", "body_to_lidar_xyz", "body_to_lidar_rpy"):
        values = mount.get(key)
        if not isinstance(values, list) or len(values) != 3 or any(
                isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value)
                for value in values):
            raise ValueError(f"MID360 {key} 必须为三个有限数值")
    fastlio = _parameters(fastlio_path, "/**")
    common = fastlio["common"]
    if common["lid_topic"] != "/livox/lidar" or common["imu_topic"] != "/livox/imu":
        raise ValueError("FAST-LIO2 必须使用 MID360 点云与内部 IMU")
    if fastlio["preprocess"]["lidar_type"] != 1:
        raise ValueError("FAST-LIO2 必须选择 Livox CustomMsg 输入")
    publish = fastlio["publish"]
    if publish["scan_publish_en"] is not True or publish["scan_bodyframe_pub_en"] is not True:
        raise ValueError("FAST-LIO2 必须发布 body 坐标点云供二维投影")
    translation = fastlio["mapping"]["extrinsic_T"]
    rotation = fastlio["mapping"]["extrinsic_R"]
    if len(translation) != 3 or any(abs(a - b) > 1e-5 for a, b in zip(translation, mount["body_to_lidar_xyz"])):
        raise ValueError("URDF 与 FAST-LIO2 的内部雷达平移外参不一致")
    if len(rotation) != 9 or any(abs(a - b) > 1e-5 for a, b in zip(rotation, (1, 0, 0, 0, 1, 0, 0, 0, 1))):
        raise ValueError("当前 MID360 内部旋转外参必须为单位阵；若需旋转请同步扩展校验")
    if any(abs(value) > 1e-5 for value in mount["body_to_lidar_rpy"]):
        raise ValueError("URDF 与 FAST-LIO2 的内部雷达旋转外参不一致")
    return mount
