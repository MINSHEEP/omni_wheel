"""Unit tests for deterministic compatibility checks."""

from kiwi_slam.compatibility import (
    REQUIRED_PACKAGES,
    package_checks,
    parse_os_release,
    platform_checks,
)


def test_parse_os_release_and_supported_platform():
    parsed = parse_os_release('NAME="Ubuntu"\nVERSION_ID="22.04"\n')
    checks = platform_checks(parsed, "humble")
    assert [check.level for check in checks] == ["OK", "OK"]


def test_platform_check_rejects_wrong_versions():
    checks = platform_checks({"VERSION_ID": "24.04"}, "jazzy")
    assert [check.level for check in checks] == ["ERROR", "ERROR"]


def test_runtime_ros_packages_are_available():
    checks = package_checks(REQUIRED_PACKAGES)
    missing = [check.name for check in checks if check.level == "ERROR"]
    assert not missing, f"missing ROS runtime packages: {missing}"
