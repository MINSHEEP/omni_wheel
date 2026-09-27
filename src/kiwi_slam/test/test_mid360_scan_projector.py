"""MID360 点云投影的几何、裁剪与空射线行为。"""

import math

import pytest

from kiwi_slam.mid360_scan_projector import ProjectionConfig, project_points


def config(**overrides):
    values = dict(
        base_to_body_xyz=(0.0, 0.0, 0.0),
        base_to_body_rpy=(0.0, 0.0, 0.0),
        body_to_lidar_xyz=(0.0, 0.0, 0.0),
        body_to_lidar_rpy=(0.0, 0.0, 0.0),
        beam_count=8,
    )
    values.update(overrides)
    return ProjectionConfig(**values)


def test_uses_lidar_origin_and_nearest_occluding_point():
    projector = config(
        base_to_body_xyz=(0.0, 0.0, 0.3),
        body_to_lidar_xyz=(0.2, 0.0, 0.0),
    )
    # 雷达前移 0.2 m，最近点距光学中心 1.0 m，而不是距车体中心 1.2 m。
    ranges = project_points([(2.2, 0.0, 0.2), (1.2, 0.0, 0.2)], projector)
    assert ranges[4] == pytest.approx(1.0)
    assert sum(math.isfinite(value) for value in ranges) == 1


def test_height_filter_applies_after_extrinsic_rotation():
    projector = config(base_to_body_rpy=(math.pi / 2, 0.0, 0.0))
    # 两点 body z 都为零，旋转后 base z 分别为 0.5 和 2.0 m。
    ranges = project_points([(1.0, 0.5, 0.0), (1.0, 2.0, 0.0)], projector)
    assert ranges[4] == pytest.approx(math.hypot(1.0, 0.5))
    assert sum(math.isfinite(value) for value in ranges) == 1


def test_excludes_invalid_height_and_range_without_clearing_empty_rays():
    points = [
        (1.0, 0.0, 0.09), (1.0, 0.0, 1.21),
        (0.49, 0.0, 0.5), (20.01, 0.0, 0.5),
        (math.nan, 0.0, 0.5), (1.0, math.inf, 0.5),
        (1.0, 0.0, -math.inf), ("bad", 0.0, 0.5),
    ]
    ranges = project_points(points, config())
    assert all(math.isinf(value) and value > 0 for value in ranges)


def test_range_endpoints_are_inclusive_and_pi_wraps_to_one_bin():
    ranges = project_points(
        [(0.5, 0.0, 0.10), (20.0, 0.0, 1.2),
         (-1.0, 0.0, 0.5), (-0.75, -0.0, 0.5)],
        config(),
    )
    assert ranges[4] == pytest.approx(0.5)
    assert ranges[0] == pytest.approx(0.75)
    assert sum(math.isfinite(value) for value in ranges) == 2


def test_requires_explicit_finite_calibration_and_valid_limits():
    with pytest.raises(ValueError, match="base_to_body_xyz"):
        ProjectionConfig((), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="base_to_body_rpy"):
        ProjectionConfig((0.0, 0.0, 0.0), (0.0, math.nan, 0.0),
                         (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="高度范围"):
        config(z_min=1.2, z_max=0.10)
    with pytest.raises(ValueError, match="量程范围"):
        config(range_min=20.0, range_max=0.5)
    with pytest.raises(ValueError, match="beam_count"):
        config(beam_count=0)
