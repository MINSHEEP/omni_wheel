"""Livox 点云到二维 LaserScan 投影单元测试."""

from types import SimpleNamespace

from kiwi_navigation.livox_scan_adapter import project_points


def test_project_points_keeps_nearest_point_in_each_bin():
    """同一角度束只保留最近障碍物."""
    points = [
        SimpleNamespace(x=2.0, y=0.0, z=0.0),
        SimpleNamespace(x=1.0, y=0.0, z=0.0),
    ]
    ranges = project_points(points, -1.0, 1.0, 0.5, 0.1, 5.0, -0.2, 0.2)
    assert min(ranges) == 1.0


def test_project_points_filters_height_and_range():
    """地面、顶部和量程外点不能进入二维扫描."""
    points = [
        SimpleNamespace(x=1.0, y=0.0, z=1.0),
        SimpleNamespace(x=0.05, y=0.0, z=0.0),
        SimpleNamespace(x=2.0, y=0.0, z=0.0),
    ]
    ranges = project_points(points, -1.0, 1.0, 0.5, 0.1, 1.5, -0.2, 0.2)
    assert all(value != 2.0 for value in ranges)
    assert all(value == float("inf") for value in ranges)
