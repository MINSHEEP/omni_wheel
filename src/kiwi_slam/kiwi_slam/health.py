"""Pure health calculations shared by the monitor and unit tests."""

from __future__ import annotations

import math
from collections.abc import Iterable


def valid_range_ratio(ranges: Iterable[float], minimum: float, maximum: float) -> float:
    """Return the fraction of finite laser ranges inside the configured limits."""
    values = list(ranges)
    if not values or minimum < 0.0 or maximum <= minimum:
        return 0.0
    valid = sum(1 for value in values if math.isfinite(value) and minimum <= value <= maximum)
    return valid / len(values)


def sample_rate_hz(receipt_times: Iterable[float]) -> float:
    """Estimate message rate from monotonic receipt timestamps."""
    values = list(receipt_times)
    if len(values) < 2:
        return 0.0
    duration = values[-1] - values[0]
    return (len(values) - 1) / duration if duration > 0.0 else 0.0


def quaternion_norm(x: float, y: float, z: float, w: float) -> float:
    """Return quaternion length for odometry sanity checks."""
    return math.sqrt(x * x + y * y + z * z + w * w)
