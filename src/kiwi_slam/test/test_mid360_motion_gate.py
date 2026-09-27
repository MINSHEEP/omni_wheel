"""The velocity gate must fail closed on stale or impossible sensor timestamps."""

from kiwi_slam.mid360_motion_gate import sensors_fresh


def test_sensor_freshness_requires_both_recent_streams():
    assert sensors_fresh(10.0, 9.8, 9.9, 0.5)
    assert not sensors_fresh(10.0, None, 9.9, 0.5)
    assert not sensors_fresh(10.0, 9.8, None, 0.5)
    assert not sensors_fresh(10.0, 9.0, 9.9, 0.5)
    assert not sensors_fresh(10.0, 9.8, 9.0, 0.5)
    assert not sensors_fresh(10.0, 10.2, 9.9, 0.5)
