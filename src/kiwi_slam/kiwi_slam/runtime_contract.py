"""Kiwi 运行时工作区与统一底盘参数契约检查。"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable

import yaml
from ament_index_python.packages import get_package_prefix


class RuntimeContractError(RuntimeError):
    """运行时包来源或参数文件不满足生产链路契约。"""


def workspace_install_root(prefix: str, package_name: str) -> Path:
    """把 colcon 的隔离/合并安装前缀统一还原为工作区 install 目录。"""
    resolved = Path(prefix).resolve()
    # 默认的隔离安装形如 ``install/kiwi_base``；合并安装的前缀直接是
    # ``install``。统一后的路径可用于判断两个同名包是否来自同一工作区。
    if resolved.name == package_name and resolved.parent.name == "install":
        return resolved.parent
    return resolved


def validate_workspace_overlay(
    anchor_package: str = "kiwi_slam",
    required_packages: Iterable[str] = ("kiwi_base",),
) -> Path:
    """拒绝把当前集成包与其他工作区中的同名底盘包混在一起运行。"""
    anchor_prefix = get_package_prefix(anchor_package)
    expected_root = workspace_install_root(anchor_prefix, anchor_package)
    mismatches = []
    for package_name in required_packages:
        prefix = get_package_prefix(package_name)
        actual_root = workspace_install_root(prefix, package_name)
        if actual_root != expected_root:
            mismatches.append(f"{package_name}={prefix}")

    if mismatches:
        details = ", ".join(mismatches)
        raise RuntimeContractError(
            "检测到 ROS 2 工作区混用："
            f"{anchor_package}={anchor_prefix}, {details}。"
            "请在当前终端执行 `source <目标工作区>/tools/activate_workspace.sh`，"
            "再用 `ros2 pkg prefix kiwi_base` 确认包来源。"
        )
    return expected_root


def _require_parameters(data: object, node_name: str, names: Iterable[str]) -> dict:
    """读取节点参数字典，并为缺失字段生成可定位的中文错误。"""
    try:
        parameters = data[node_name]["ros__parameters"]  # type: ignore[index]
    except (KeyError, TypeError) as exc:
        raise RuntimeContractError(
            f"统一底盘参数缺少 `{node_name}.ros__parameters`；"
            "这通常表示加载了旧工作区中的 kiwi_base.yaml。"
        ) from exc

    if not isinstance(parameters, dict):
        raise RuntimeContractError(f"`{node_name}.ros__parameters` 必须是参数映射。")
    missing = sorted(name for name in names if name not in parameters)
    if missing:
        raise RuntimeContractError(
            f"`{node_name}` 缺少必填参数：{', '.join(missing)}。"
        )
    return parameters


def validate_base_parameters(path: str | Path) -> Path:
    """验证底盘和 EKF 确实读取同一份完整 YAML，而不是静默使用默认值。"""
    parameter_path = Path(path).expanduser().resolve()
    try:
        data = yaml.safe_load(parameter_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise RuntimeContractError(
            f"无法读取统一底盘参数 `{parameter_path}`：{exc}"
        ) from exc

    base = _require_parameters(
        data,
        "kiwi_base_node",
        (
            "publish_rate",
            "feedback_framed",
            "cmd_vel_timeout_s",
            "wheel_odom_topic",
            "imu_topic",
            "odom_frame_id",
            "base_frame_id",
        ),
    )
    ekf = _require_parameters(
        data,
        "ekf_filter_node",
        (
            "frequency",
            "publish_tf",
            "odom0",
            "imu0",
            "odom_frame",
            "base_link_frame",
            "world_frame",
        ),
    )

    # 这些相等关系是 EKF 能生成 odom -> base_footprint 的最低契约。启动前
    # 检查可把“节点存在但一直零输出”转换为直接、可定位的启动错误。
    if base["wheel_odom_topic"] != ekf["odom0"]:
        raise RuntimeContractError("底盘 wheel_odom_topic 与 EKF odom0 不一致。")
    if base["imu_topic"] != ekf["imu0"]:
        raise RuntimeContractError("底盘 imu_topic 与 EKF imu0 不一致。")
    if (
        base["odom_frame_id"] != ekf["odom_frame"]
        or ekf["world_frame"] != ekf["odom_frame"]
    ):
        raise RuntimeContractError("底盘与 EKF 的 odom/world frame 不一致。")
    if base["base_frame_id"] != ekf["base_link_frame"]:
        raise RuntimeContractError("底盘与 EKF 的 base frame 不一致。")
    if ekf["publish_tf"] is not True:
        raise RuntimeContractError("EKF publish_tf 必须为 true。")

    for node_name, parameter_name, value in (
        ("kiwi_base_node", "publish_rate", base["publish_rate"]),
        ("kiwi_base_node", "cmd_vel_timeout_s", base["cmd_vel_timeout_s"]),
        ("ekf_filter_node", "frequency", ekf["frequency"]),
    ):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise RuntimeContractError(f"{node_name}.{parameter_name} 必须是有限正数。")
        if not math.isfinite(float(value)) or float(value) <= 0.0:
            raise RuntimeContractError(f"{node_name}.{parameter_name} 必须是有限正数。")

    return parameter_path


def validate_tracked_laser_parameters(
    base_path: str | Path,
    lidar_path: str | Path,
    rf2o_path: str | Path,
    gate_path: str | Path,
    slam_path: str | Path,
) -> None:
    """启动前核对双履带、M10P、RF2O 和 SLAM 的单一里程计契约。"""
    def load(path: str | Path) -> dict:
        parameter_path = Path(path).expanduser().resolve()
        try:
            data = yaml.safe_load(parameter_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, yaml.YAMLError) as exc:
            raise RuntimeContractError(f"无法读取参数 `{parameter_path}`：{exc}") from exc
        if not isinstance(data, dict):
            raise RuntimeContractError(f"参数 `{parameter_path}` 必须是节点映射。")
        return data

    base_data = load(base_path)
    base = _require_parameters(
        base_data, "tracked_base_node",
        ("track_separation_m", "command_per_m_s", "max_motor_command",
         "hardware_calibrated", "enable_motion", "cmd_vel_timeout_s"),
    )
    _require_parameters(base_data, "tracked_robot_model", ("lidar_xyz", "lidar_rpy"))
    lidar = _require_parameters(
        load(lidar_path), "lslidar_driver_node", ("laserscan_topic", "frame_id")
    )
    rf2o = _require_parameters(
        load(rf2o_path), "rf2o_laser_odometry",
        ("laser_scan_topic", "odom_topic", "publish_tf", "base_frame_id",
         "odom_frame_id", "init_pose_from_topic", "freq"),
    )
    gate = _require_parameters(
        load(gate_path), "kiwi_mapping_scan_gate",
        ("input_scan_topic", "output_scan_topic", "odom_topic",
         "require_imu_ready", "laser_frame", "odom_frame", "base_frame"),
    )
    slam = _require_parameters(
        load(slam_path), "slam_toolbox",
        ("scan_topic", "odom_frame", "base_frame", "map_frame"),
    )
    if base["enable_motion"] is True and base["hardware_calibrated"] is not True:
        raise RuntimeContractError("履带运动已启用，但硬件标定未确认。")
    for field in ("track_separation_m", "command_per_m_s", "max_motor_command",
                  "cmd_vel_timeout_s"):
        value = base[field]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or \
                not math.isfinite(float(value)) or value <= 0.0:
            raise RuntimeContractError(f"tracked_base_node.{field} 必须是有限正数。")
    if rf2o["laser_scan_topic"] != lidar["laserscan_topic"] or \
            gate["input_scan_topic"] != lidar["laserscan_topic"]:
        raise RuntimeContractError("M10P、RF2O 和建图门控的扫描话题不一致。")
    if rf2o["odom_topic"] != gate["odom_topic"] or \
            rf2o["odom_topic"] != "/odometry/filtered":
        raise RuntimeContractError("RF2O 必须是 /odometry/filtered 的唯一来源。")
    if rf2o["publish_tf"] is not True or rf2o["init_pose_from_topic"] != "":
        raise RuntimeContractError("RF2O 必须发布 odom TF，并从原点自行初始化。")
    frequency = rf2o["freq"]
    if isinstance(frequency, bool) or not isinstance(frequency, (int, float)) or \
            not math.isfinite(float(frequency)) or frequency <= 0.0:
        raise RuntimeContractError("RF2O 频率必须是有限正数。")
    if gate["require_imu_ready"] is not False:
        raise RuntimeContractError("纯激光建图不能等待 IMU 标定。")
    if gate["output_scan_topic"] != slam["scan_topic"] or \
            gate["laser_frame"] != lidar["frame_id"]:
        raise RuntimeContractError("SLAM 扫描话题或雷达坐标系不一致。")
    if rf2o["odom_frame_id"] != gate["odom_frame"] or \
            rf2o["odom_frame_id"] != slam["odom_frame"] or \
            rf2o["base_frame_id"] != gate["base_frame"] or \
            rf2o["base_frame_id"] != slam["base_frame"]:
        raise RuntimeContractError("RF2O、建图门控和 SLAM 的 odom/base 坐标系不一致。")
