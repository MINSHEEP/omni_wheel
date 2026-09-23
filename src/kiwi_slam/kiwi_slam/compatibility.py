"""ROS 2 Humble and Ubuntu 22.04 compatibility checks for Kiwi mapping."""

from __future__ import annotations

import argparse
import os
from dataclasses import dataclass
from pathlib import Path

import yaml
from kiwi_platform import load_profile
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
)


REQUIRED_PACKAGES = (
    "kiwi_platform",
    "kiwi_base",
    "lslidar_driver",
    "lslidar_msgs",
    "robot_localization",
    "robot_state_publisher",
    "slam_toolbox",
    "tf2_ros",
    "xacro",
)


@dataclass(frozen=True)
class Check:
    """One compatibility result with a stable severity label."""

    level: str
    name: str
    message: str


def parse_os_release(text: str) -> dict[str, str]:
    """Parse /etc/os-release without shell commands."""
    result: dict[str, str] = {}
    for line in text.splitlines():
        if "=" not in line or line.startswith("#"):
            continue
        key, raw = line.split("=", 1)
        result[key] = raw.strip().strip('"')
    return result


def platform_checks(os_release: dict[str, str], ros_distro: str) -> list[Check]:
    """Validate the supported operating system and ROS distribution."""
    checks = []
    version = os_release.get("VERSION_ID", "unknown")
    checks.append(
        Check("OK" if version == "22.04" else "ERROR", "ubuntu", f"VERSION_ID={version}")
    )
    checks.append(
        Check("OK" if ros_distro == "humble" else "ERROR", "ros", f"ROS_DISTRO={ros_distro}")
    )
    return checks


def package_checks(names: tuple[str, ...] = REQUIRED_PACKAGES) -> list[Check]:
    """Check runtime packages through the active ament index."""
    checks = []
    for name in names:
        try:
            prefix = get_package_prefix(name)
            checks.append(Check("OK", f"package:{name}", prefix))
        except PackageNotFoundError:
            checks.append(Check("ERROR", f"package:{name}", "not found in ament index"))
    return checks


def device_check(path: str, required: bool) -> Check:
    """Check serial device existence and read/write access."""
    device = Path(path)
    if not device.exists():
        return Check("ERROR" if required else "WARN", f"device:{path}", "not found")
    access = os.access(device, os.R_OK | os.W_OK)
    message = "read/write" if access else "permission denied"
    return Check("OK" if access else "ERROR", f"device:{path}", message)


def load_parameter(path: Path, node: str, name: str) -> object:
    """从 ROS 参数 YAML 读取单个必填字段。"""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    try:
        return data[node]["ros__parameters"][name]
    except (KeyError, TypeError) as exc:
        raise ValueError(f"missing {node}.ros__parameters.{name} in {path}") from exc


def run_checks(base_device: str, lidar_port: str, require_hardware: bool) -> list[Check]:
    """Collect platform, package, and serial-device checks."""
    try:
        os_release = parse_os_release(Path("/etc/os-release").read_text(encoding="utf-8"))
    except OSError:
        os_release = {}
    checks = platform_checks(os_release, os.environ.get("ROS_DISTRO", ""))
    checks.extend(package_checks())
    checks.append(device_check(base_device, require_hardware))
    checks.append(device_check(lidar_port, require_hardware))
    return checks


def main(argv: list[str] | None = None) -> int:
    """Print a concise compatibility report and return nonzero on errors."""
    parser = argparse.ArgumentParser(description="Check Kiwi SLAM runtime compatibility")
    parser.add_argument("--base-params-file", type=Path)
    parser.add_argument("--lidar-params-file", type=Path)
    parser.add_argument("--require-hardware", action="store_true")
    args = parser.parse_args(argv)

    profile = load_profile()
    base_file = args.base_params_file or Path(profile.path("base"))
    lidar_file = args.lidar_params_file or Path(profile.path("lidar"))
    base_device = str(load_parameter(base_file, profile.driver, "device"))
    # M10P 串口统一读取 lslidar_driver_node.serial_port，避免检查错配置层级。
    lidar_port = str(
        load_parameter(lidar_file, "lslidar_driver_node", "serial_port")
    )
    checks = run_checks(base_device, lidar_port, args.require_hardware)
    if profile.odometry == "rf2o":
        checks.extend(package_checks(("rf2o_laser_odometry",)))
    for check in checks:
        print(f"[{check.level:5}] {check.name}: {check.message}")
    errors = sum(check.level == "ERROR" for check in checks)
    print(f"summary: {len(checks) - errors}/{len(checks)} checks without error")
    return 1 if errors else 0
