"""Unit tests for hardware-independent health calculations."""

import math

import pytest

from kiwi_slam.health import quaternion_norm, sample_rate_hz, valid_range_ratio


def test_valid_range_ratio_rejects_nan_inf_and_out_of_range():
    ranges = [0.05, 1.0, 12.0, math.inf, math.nan, 0.01, 13.0]
    assert valid_range_ratio(ranges, 0.05, 12.0) == 3 / 7


def test_valid_range_ratio_rejects_invalid_configuration():
    assert valid_range_ratio([], 0.05, 12.0) == 0.0
    assert valid_range_ratio([1.0], 12.0, 0.05) == 0.0


def test_sample_rate_and_quaternion_norm():
    assert sample_rate_hz([1.0, 1.1, 1.2]) == pytest.approx(10.0)
    assert sample_rate_hz([1.0]) == 0.0
    assert quaternion_norm(0.0, 0.0, 0.0, 1.0) == 1.0
