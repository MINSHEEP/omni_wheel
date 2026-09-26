"""独立惯性预测的运动、时限和异步锚点契约，不需要 ROS 节点或硬件。"""
import math

import numpy as np
import pytest

from kiwi_slam.inertial_odometry import InertialConfig, InertialOdometry, gyro_at


ACCEL_COV = np.eye(2)*.04
POSE_COV = np.diag([.25, .25, .01])


def test_node_heartbeat_readers_require_fresh_messages_and_status_remains_latched(monkeypatch):
    import rclpy
    import rclpy.node
    from types import SimpleNamespace
    from unittest.mock import Mock
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    from kiwi_slam.inertial_odometry import main

    readers, writers = {}, {}
    class NodeStub:
        def __init__(self, name):
            pass
        def declare_parameter(self, name, default):
            return SimpleNamespace(value=default)
        def create_subscription(self, message_type, topic, callback, qos):
            readers[topic] = qos
            return Mock()
        def create_publisher(self, message_type, topic, qos):
            writers[topic] = qos
            return Mock()
        def create_timer(self, *args):
            return Mock()
        def get_logger(self):
            return Mock()
        def destroy_node(self):
            pass
    monkeypatch.setattr(rclpy.node, 'Node', NodeStub)
    for name in ('init', 'spin', 'shutdown'):
        monkeypatch.setattr(rclpy, name, Mock())
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    main([])
    for topic in ('/chassis/imu/data_ready', '/chassis/odom/laser_ready',
                  '/chassis/odom/laser_partial_ready'):
        qos = readers[topic]
        assert qos.depth == 10
        assert qos.reliability == ReliabilityPolicy.RELIABLE
        assert qos.durability == DurabilityPolicy.VOLATILE
    for topic in ('/chassis/inertial/valid', '/chassis/inertial/state',
                  '/chassis/inertial/reason'):
        assert writers[topic].durability == DurabilityPolicy.TRANSIENT_LOCAL
        assert writers[topic].depth == 1


def push(core, stamp, acceleration=(0., 0.), rate=0.):
    return core.push_imu(stamp, acceleration, rate, ACCEL_COV, .001)


def primed(speed=.2, angle=0., config=None, pose_covariance=POSE_COV):
    core = InertialOdometry(config)
    for i in range(21):
        stamp = 10.+i*.02
        push(core, stamp)
        if i % 5 == 0:
            core.add_anchor(stamp, [speed*(stamp-10.), 0., angle], pose_covariance)
    assert core.estimate(10.4).valid
    return core


def test_constant_speed_does_not_become_stationary_when_laser_is_missing():
    for speed in (-.25, .25):
        core = primed(speed)
        for i in range(1, 81):
            push(core, 10.4+i*.02)
        result = core.estimate(12.)
        assert result.valid and result.reason == 'coasting'
        assert result.anchor_stamp == pytest.approx(10.4)
        assert result.state[0] == pytest.approx(speed*2., abs=1e-10)
        assert result.state[3] == pytest.approx(speed, abs=1e-10)
        assert result.covariance[0, 0] > POSE_COV[0, 0]
        assert np.linalg.eigvalsh(result.covariance).min() >= 0


def test_calibrated_body_acceleration_rotates_into_odom_without_second_gravity_removal():
    core = primed(0., math.pi/2)
    for i in range(1, 51):
        push(core, 10.4+i*.02, (.3, 0.))
    result = core.estimate(11.4)
    assert result.valid
    assert abs(result.state[0]) < 1e-10
    assert result.state[1] == pytest.approx(.15, abs=.004)
    assert result.state[4] == pytest.approx(.3, abs=.004)


def test_rotation_in_place_integrates_yaw_without_inventing_translation():
    core = primed(0.)
    for i in range(1, 51):
        push(core, 10.4+i*.02, rate=.5)
    result = core.estimate(11.4)
    assert result.valid
    assert result.state[2] == pytest.approx(.5, abs=.006)
    assert np.linalg.norm(result.state[[0, 1, 3, 4]]) < 1e-10


def test_prediction_expires_and_does_not_refresh_itself_or_integrate_forever():
    core = primed(.2)
    for i in range(1, 201):
        push(core, 10.4+i*.02)
    result = core.estimate(14.4)
    assert not result.valid and result.reason == 'coast_timeout'
    assert result.anchor_stamp == pytest.approx(10.4)
    assert result.stamp <= 12.4+1e-8
    assert result.state[0] <= .48+1e-8


def test_imu_gap_requires_new_position_and_velocity_anchors():
    core = primed()
    assert not push(core, 11.)
    assert core.estimate(11.) is None
    assert core.reason == 'imu_gap_requires_new_anchor'
    for i in range(1, 5):
        push(core, 11.+i*.02)
    assert core.estimate(11.08) is None
    # 一帧位置不能恢复未知速度，必须等新的持续多帧可信观测。
    assert not core.add_anchor(11.08, [.216, 0., 0.], POSE_COV)
    assert core.estimate(11.08) is None


def test_missing_imu_invalidates_even_when_wall_timer_keeps_running():
    core = primed()
    result = core.estimate(10.65)
    assert not result.valid and result.reason == 'imu_stale'


def test_delayed_twelve_hz_laser_replays_fifty_hz_imu_at_sample_time():
    core = InertialOdometry()
    events = []
    for i in range(151):
        stamp = 1000.+i*.02
        events.append((stamp, 'imu', stamp))
    for i in range(36):
        stamp = 1000.+i/12.
        events.append((stamp+.06, 'laser', stamp))
    last_stamp = -math.inf
    ready = 0
    for received, kind, stamp in sorted(events):
        if kind == 'imu':
            push(core, stamp)
        else:
            core.add_anchor(stamp, [.3*(stamp-1000.), 0., 0.], POSE_COV)
        result = core.estimate(received)
        if result is not None and result.valid:
            assert result.stamp >= last_stamp
            last_stamp = result.stamp
            assert result.state[0] == pytest.approx(.3*(result.stamp-1000.), abs=1e-9)
            assert result.state[3] == pytest.approx(.3, abs=1e-9)
            ready += 1
    assert ready > 100


def test_laser_arriving_before_corresponding_imu_waits_without_reversing_time():
    core = primed()
    assert not core.add_anchor(10.5, [.10, 0., 0.], POSE_COV)
    assert core.estimate(10.4).stamp == pytest.approx(10.4)
    for i in range(1, 7):
        push(core, 10.4+i*.02)
    result = core.estimate(10.52)
    assert result.anchor_stamp == pytest.approx(10.5)
    assert result.stamp == pytest.approx(10.52)
    assert result.state[0] == pytest.approx(.104)


def test_pose_correction_jump_does_not_create_velocity_or_extend_coast():
    core = primed(.2)
    assert not core.add_anchor(10.5, [.50, 0., 0.], POSE_COV)
    assert core.last_anchor_rejection == 'anchor_jump_velocity_unverified'
    push(core, 10.5)
    result = core.estimate(10.5)
    assert result.anchor_stamp == pytest.approx(10.4)
    assert result.state[3] == pytest.approx(.2)
    assert result.state[0] == pytest.approx(.10)


def test_new_trusted_absolute_laser_anchors_can_recover_after_timeout():
    core = primed(.2)
    for i in range(1, 151):
        stamp = 10.4+i*.02
        push(core, stamp)
        if i >= 130 and i % 5 == 0:
            core.add_anchor(stamp, [.2*(stamp-10.), 0., 0.], POSE_COV)
    result = core.estimate(13.4)
    assert result.valid
    assert result.state[0] == pytest.approx(.68)
    assert result.state[3] == pytest.approx(.2)
    assert result.anchor_stamp == pytest.approx(13.4)


def test_uncertainty_limit_is_independent_of_time_limit_and_covariance_stays_positive():
    config = InertialConfig(max_position_variance=.27)
    core = primed(config=config)
    assert core.estimate(10.4).valid  # 实际激光初始 xy 方差为 0.25。
    for i in range(1, 91):
        push(core, 10.4+i*.02)
    result = core.estimate(12.2)
    assert not result.valid and result.reason == 'position_uncertainty_limit'
    assert np.linalg.eigvalsh(result.covariance).min() >= -1e-10


def test_large_laser_covariance_is_preserved_while_additional_inertial_error_is_bounded():
    initial = np.diag([2., 2.5, .01])
    core = primed(config=InertialConfig(max_position_growth_variance=.015),
                  pose_covariance=initial)
    anchored = core.estimate(10.4)
    assert anchored.valid
    assert np.allclose(anchored.covariance[:2, :2], initial[:2, :2])
    for i in range(1, 81):
        push(core, 10.4+i*.02)
    result = core.estimate(12.)
    assert not result.valid and result.reason == 'position_uncertainty_growth_limit'
    assert result.covariance[0, 0] >= 2.
    assert result.covariance[1, 1] >= 2.5
    assert np.allclose(core.anchor_position_covariance, initial[:2, :2])


def test_acceleration_shock_invalid_covariance_and_backwards_clock_cannot_output_pose():
    for acceleration, covariance, stamp in [([2., 0.], ACCEL_COV, 10.42),
                                             ([0., 0.], -ACCEL_COV, 10.42),
                                             ([0., 0.], ACCEL_COV, 8.)]:
        core = primed()
        assert not core.push_imu(stamp, acceleration, 0., covariance, .001)
        assert core.estimate(stamp) is None


def test_timestamp_matched_gyro_interpolation_does_not_take_arbitrary_latest_sample():
    samples = [(1., .1, .01), (1.02, .2, .02), (1.04, .3, .03)]
    assert gyro_at(samples, 1.01, .025) == pytest.approx((.15, .015))
    assert gyro_at(samples, 1.07, .025) is None
    assert gyro_at(samples, .9, .025) is None


def test_noisy_position_window_does_not_claim_confident_speed():
    core = InertialOdometry()
    accepted = []
    for i in range(26):
        stamp = 10.+i*.02
        push(core, stamp)
        if i % 5 == 0:
            # 小于物理跳变阈值、但不能由稳定速度解释的交替位置扰动。
            noise = .025 if i % 10 == 0 else -.025
            accepted.append(core.add_anchor(stamp, [noise, 0., 0.], POSE_COV))
    assert not any(accepted)
    assert core.estimate(10.5) is None


def test_invalid_or_unbounded_configuration_is_rejected():
    for values in ({'max_coast_s': 60.}, {'max_imu_gap_s': 0.},
                   {'anchor_min_samples': 2}, {'anchor_window_s': .1},
                   {'velocity_variance_floor': .8}):
        with pytest.raises(ValueError):
            InertialConfig(**values)


@pytest.mark.parametrize(('acceleration', 'initial_velocity'), [(0.3, 0.), (-0.3, .6)])
def test_anchor_velocity_is_terminal_velocity_during_acceleration_and_braking(
        acceleration, initial_velocity):
    core = InertialOdometry()
    for i in range(101):
        t = i * .02
        push(core, 10. + t, (acceleration, 0.))
        if i <= 20 and i % 5 == 0:
            core.add_anchor(10. + t,
                            [initial_velocity*t + .5*acceleration*t*t, 0., 0.], POSE_COV)
        if i == 20:
            anchor = core.estimate(10.4)
            assert anchor.state[3] == pytest.approx(initial_velocity + acceleration*.4, abs=1e-10)
            assert np.allclose(anchor.covariance[:3, :3], POSE_COV)
    result = core.estimate(12.)
    assert result.valid and result.anchor_stamp == pytest.approx(10.4)
    assert result.state[0] == pytest.approx(initial_velocity*2. + .5*acceleration*4., abs=1e-10)
    assert result.state[3] == pytest.approx(initial_velocity + acceleration*2., abs=1e-10)
    # 原窗口平均速度在这里产生 ±0.096 m 位置误差；不能靠缩短时限掩盖。
    assert abs(acceleration * .2 * 1.6) == pytest.approx(.096)


def test_delayed_turning_anchors_compensate_acceleration_in_the_matching_body_frame():
    core = InertialOdometry()
    speed, rate = .3, .6
    radius = speed/rate
    events = [(10.+i*.02, 'imu', 10.+i*.02) for i in range(126)]
    events += [(10.+i/12.+.08, 'anchor', 10.+i/12.) for i in range(13)]
    for receipt, kind, stamp in sorted(events):
        t = stamp - 10.
        if kind == 'imu':
            push(core, stamp, (0., speed*rate), rate)
        else:
            core.add_anchor(stamp, [radius*math.sin(rate*t),
                                   radius*(1.-math.cos(rate*t)), rate*t], POSE_COV)
    result = core.estimate(12.5)
    expected = np.array([radius*math.sin(rate*2.5), radius*(1.-math.cos(rate*2.5)),
                         rate*2.5, speed*math.cos(rate*2.5), speed*math.sin(rate*2.5)])
    assert result.valid and result.anchor_stamp == pytest.approx(11.)
    assert np.max(np.abs(result.state-expected)) < 2e-5
    assert np.linalg.eigvalsh(result.covariance).min() >= -1e-10


def test_irregular_imu_sampling_and_early_anchor_do_not_use_unobserved_future_acceleration():
    core = InertialOdometry()
    acceleration = .25
    events = [(10.+t, 'imu', 10.+t) for t in [0., .03, .08, .16, .24, .32, .41, .5, .6]]
    # t=0.1、0.2 等激光先到，须等后一个 IMU 样本后才计算并重放。
    events += [(10.+i*.1, 'anchor', 10.+i*.1) for i in range(5)]
    for receipt, kind, stamp in sorted(events):
        if kind == 'imu':
            push(core, stamp, (acceleration, 0.))
        else:
            t = stamp - 10.
            core.add_anchor(stamp, [.5*acceleration*t*t, 0., 0.], POSE_COV)
    result = core.estimate(10.6)
    assert result.valid and result.anchor_stamp == pytest.approx(10.4)
    assert result.state[0] == pytest.approx(.5*acceleration*.6**2, abs=1e-10)
    assert result.state[3] == pytest.approx(acceleration*.6, abs=1e-10)
    # 大于允许值的真实 IMU 缺口不能由插值补造数据。
    push(core, 10.9, (acceleration, 0.))
    assert core.estimate(10.9) is None
    assert core.reason == 'imu_gap_requires_new_anchor'


def test_imu_does_not_cover_velocity_window_cannot_create_a_velocity_anchor():
    core = InertialOdometry()
    push(core, 10.4)
    for i in range(5):
        t = i*.1
        core.add_anchor(10.+t, [.2*t, 0., 0.], POSE_COV)
    assert core.estimate(10.4) is None
    assert core.last_anchor_rejection == 'anchor_velocity_imu_history_incomplete'


def test_compensation_noise_is_counted_without_reducing_inherited_pose_uncertainty():
    variances = []
    for noise in (.04, .4):
        core = InertialOdometry()
        for i in range(21):
            t = i*.02
            core.push_imu(10.+t, [.3, 0.], 0., np.eye(2)*noise, .001)
            if i % 5 == 0:
                core.add_anchor(10.+t, [.15*t*t, 0., 0.], POSE_COV)
        estimate = core.estimate(10.4)
        assert estimate.valid
        assert np.allclose(estimate.covariance[:3, :3], POSE_COV)
        variances.append(estimate.covariance[3, 3])
    assert variances[1] > variances[0] > core.config.velocity_variance_floor


def test_large_inherited_absolute_uncertainty_does_not_disable_local_inertial_bridge():
    initial = np.diag([12.5165788511404, 2., .01])
    core = primed(.2, pose_covariance=initial)
    anchor = core.estimate(10.4)
    assert anchor.valid
    assert np.allclose(anchor.covariance[:3, :3], initial)
    for i in range(1, 51):
        push(core, 10.4+i*.02)
    estimate = core.estimate(11.4)
    assert estimate.valid
    assert estimate.covariance[0, 0] > 12.5165788511404
    assert np.linalg.eigvalsh(estimate.covariance[:2, :2]-initial[:2, :2]).max() < .09
    assert estimate.anchor_stamp == pytest.approx(10.4)
    assert not core.estimate(12.41).valid


def test_partial_velocity_corrects_only_observable_direction_without_reanchoring():
    core = primed(.2)
    for i in range(1, 11):
        push(core, 10.4+i*.02, (0., .1))
    before = core.estimate(10.6)
    # 沿走廊的 vx 未观测，不能因为报文是 0 就强行停止恒速前进。
    assert core.add_partial_velocity(10.6, [0., 0.], np.diag([1e6, .01]))
    after = core.estimate(10.6)
    assert after.state[3] == pytest.approx(before.state[3], abs=1e-12)
    assert 0 < after.state[4] < before.state[4]
    assert np.allclose(after.state[:3], before.state[:3])
    assert np.linalg.eigvalsh(after.covariance-before.covariance).min() >= -1e-12
    assert after.anchor_stamp == pytest.approx(10.4)
    for i in range(11, 106):
        stamp = 10.4+i*.02
        push(core, stamp)
        core.add_partial_velocity(stamp, [0., 0.], np.diag([1e6, .01]))
    assert not core.estimate(12.5).valid
    assert core.anchor_stamp == pytest.approx(10.4)


def test_partial_velocity_uses_full_correlated_covariance_in_body_frame():
    angle = math.pi/4
    core = primed(.2, angle)
    for i in range(1, 11):
        push(core, 10.4+i*.02, (.1/math.sqrt(2), .1/math.sqrt(2)))
    observable = np.array([1., 1.])/math.sqrt(2)
    weak = np.array([-1., 1.])/math.sqrt(2)
    covariance = .01*np.outer(observable, observable) + 1e6*np.outer(weak, weak)
    body_velocity = [.2*math.cos(angle), -.2*math.sin(angle)]
    before = core.estimate(10.6)
    assert core.add_partial_velocity(10.6, body_velocity, covariance)
    after = core.estimate(10.6)
    assert after.state[3] == pytest.approx(before.state[3], abs=1e-10)
    assert 0 < after.state[4] < before.state[4]
    assert np.linalg.eigvalsh(after.covariance).min() >= -1e-10


def test_delayed_partial_velocity_during_turn_does_not_create_fictitious_lateral_motion():
    core = InertialOdometry()
    speed, rate = .3, .6
    radius = speed/rate
    for i in range(101):
        t = i*.02
        push(core, 10.+t, (0., speed*rate), rate)
        if i <= 50 and i % 5 == 0:
            core.add_anchor(10.+t, [radius*math.sin(rate*t),
                                   radius*(1.-math.cos(rate*t)), rate*t], POSE_COV)
    before = core.estimate(12.)
    assert core.add_partial_velocity(11.92, [speed, 0.], np.diag([1e6, .01]))
    after = core.estimate(12.)
    assert np.max(np.abs(after.state-before.state)) < 2e-6
    assert after.anchor_stamp == pytest.approx(11.)
    assert np.linalg.eigvalsh(after.covariance-before.covariance).min() >= -1e-10


def test_partial_bad_covariance_outliers_duplicates_or_unobserved_axes_are_rejected():
    for stamp, velocity, covariance in [
            (10.4, [.2, 0.], np.eye(2)*1e6),
            (10.4, [.2, 0.], -np.eye(2)),
            (10.4, [.2, math.nan], np.diag([1e6, .01])),
            (10.4, [.2, .6], np.diag([1e6, .01])),
            (10.5, [.2, 0.], np.diag([1e6, .01])),
            (10.1, [.2, 0.], np.diag([1e6, .01]))]:
        core = primed(.2)
        before = core.estimate(10.4)
        assert not core.add_partial_velocity(stamp, velocity, covariance)
        after = core.estimate(10.4)
        assert np.array_equal(after.state, before.state)
        assert np.array_equal(after.covariance, before.covariance)
    assert core.add_partial_velocity(10.4, [.2, 0.], np.diag([1e6, .01]))
    assert not core.add_partial_velocity(10.4, [.2, 0.], np.diag([1e6, .01]))


def test_inherited_yaw_uncertainty_is_preserved_with_separate_growth_limit():
    covariance = np.diag([12.5165788511404, 2., .7])
    core = primed(.2, pose_covariance=covariance)
    assert core.estimate(10.4).valid
    assert core.estimate(10.4).covariance[2, 2] == pytest.approx(.7)
    config = InertialConfig(gyro_bias_std=.1, max_yaw_growth_variance=.001)
    core = primed(.2, config=config, pose_covariance=covariance)
    for i in range(1, 26):
        push(core, 10.4+i*.02)
    estimate = core.estimate(10.9)
    assert not estimate.valid
    assert estimate.reason in ('yaw_uncertainty_limit', 'yaw_uncertainty_growth_limit')
    assert estimate.covariance[2, 2] > .7
