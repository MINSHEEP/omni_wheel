"""速度中继纯函数单元测试."""

import math

from kiwi_navigation.cmd_relay import clamp_finite, sanitize_command


def test_clamp_finite_rejects_invalid_values():
    """Nan 和无穷值不能进入底盘."""
    assert clamp_finite(math.nan, 1.0) == 0.0
    assert clamp_finite(math.inf, 1.0) == 0.0
    assert clamp_finite(-math.inf, 1.0) == 0.0


def test_clamp_finite_limits_both_directions():
    """正负方向共享同一个绝对速度限值."""
    assert clamp_finite(2.0, 0.3) == 0.3
    assert clamp_finite(-2.0, 0.3) == -0.3
    assert clamp_finite(0.2, 0.3) == 0.2


def test_sanitize_command_keeps_kiwi_axes():
    """全向底盘必须同时保留 vx、vy 和 wz."""
    assert sanitize_command(0.4, -0.2, 1.2, (0.3, 0.25, 0.8)) == (
        0.3,
        -0.2,
        0.8,
    )
