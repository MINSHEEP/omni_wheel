"""Motion ownership, deadman, delayed packets and arbitration contracts."""

import math

import pytest

from kiwi_web.control import CommandGate


@pytest.fixture
def gate():
    clock = [100.0]
    control = CommandGate(clock=lambda: clock[0])
    control.mode = "remote"
    control.manual_ready = True
    return control, clock


def test_deadman_disconnect_and_new_owner_do_not_resume_old_velocity(gate):
    control, clock = gate
    control.acquire("a")
    control.drive("a", [0.3, 0, 0], 0, control.challenge())
    assert control.output() == (0.3, 0, 0)
    clock[0] += 0.26
    assert control.output() == (0, 0, 0)
    control.release("a")
    control.acquire("b")
    assert control.output() == (0, 0, 0)
    with pytest.raises(ValueError):
        control.drive("a", [0.3, 0, 0], 1, control.challenge())


def test_owner_mutual_exclusion_and_lease_expiry(gate):
    control, clock = gate
    control.acquire("a")
    with pytest.raises(ValueError):
        control.acquire("b")
    control.release("b")
    assert control.status()["owner"] == "a"
    clock[0] += 1.01
    control.acquire("b")
    assert control.status()["owner"] == "b"


def test_stale_reordered_and_nonfinite_commands_are_rejected(gate):
    control, clock = gate
    control.acquire("a")
    challenge = control.challenge()
    control.drive("a", [0, 0, 0], 2, challenge)
    with pytest.raises(ValueError):
        control.drive("a", [0.3, 0, 0], 1, challenge)
    for value in (math.nan, math.inf, "1", True):
        with pytest.raises(ValueError):
            control.drive("a", [value, 0, 0], 3, challenge)
    clock[0] += 0.31
    with pytest.raises(ValueError):
        control.drive("a", [0.3, 0, 0], 3, challenge)
    assert control.output() == (0, 0, 0)


def test_estop_latches_and_reset_never_resumes(gate):
    control, _ = gate
    control.mode = "navigation"
    control.nav_ready = control.nav_enabled = True
    control.navigation([0.3, 0, 0.3])
    assert control.output() == (0.3, 0, 0.3)
    control.stop(latch=True)
    assert control.output() == (0, 0, 0)
    control.reset()
    assert control.output() == (0, 0, 0)
    assert not control.nav_enabled


def test_health_mode_and_proportional_limits(gate):
    control, _ = gate
    control.acquire("a")
    control.drive("a", [1.6, 0.8, 0.6], 0, control.challenge())
    assert control.output() == (0.4, 0.2, 0.15)
    control.manual_ready = False
    assert control.output() == (0, 0, 0)
    control.release()
    control.mode = "navigation"
    with pytest.raises(ValueError):
        control.acquire("a")


def test_navigation_keeps_its_own_velocity_limits():
    control = CommandGate(angular=0.4, navigation_limits=(0.8, 0.8, 1.5))
    control.mode = "navigation"
    control.nav_enabled = control.nav_ready = True
    control.navigation([0.3, 0, 0.8])
    assert control.output() == (0.3, 0, 0.8)


def test_tracked_zero_lateral_limit_rejects_sideways_motion():
    control = CommandGate(linear=0.2, lateral=0.0, angular=0.4,
                          navigation_limits=(0.2, 0.0, 0.4))
    control.mode = "remote"
    control.manual_ready = True
    control.acquire("operator")
    control.drive("operator", [0.1, 0.0, 0.2], 0, control.challenge())
    assert control.output() == (0.1, 0.0, 0.2)
    with pytest.raises(ValueError, match="不支持"):
        control.drive("operator", [0.0, 0.1, 0.0], 1, control.challenge())
    control.mode = "navigation"
    with pytest.raises(ValueError, match="不支持"):
        control.navigation([0.0, -0.1, 0.0])
