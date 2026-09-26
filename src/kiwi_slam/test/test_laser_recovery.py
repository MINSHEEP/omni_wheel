"""恢复必须保留原坐标、累计运动和不确定度，不能只换一帧扫描。"""
import math
import numpy as np

from kiwi_slam.laser_quality import Match, ScanEvidence, cloud
from kiwi_slam.reference_recovery import ReferenceRecovery, relative_pose, compose_pose
from kiwi_slam.odom_covariance import preserve_covariance_floor
from test_tracked_estimation import scan


def inertial(stamp, pose, covariance=None):
    return dict(stamp=stamp, pose=np.asarray(pose),
                covariance=np.diag([.5, .6, .08]) if covariance is None else covariance)


def test_local_matching_recovers_geometry_without_replacing_absolute_reference():
    evidence = ScanEvidence()
    for i in range(31):
        evidence.observe(scan(), i*.1, 0.)
    points, normals = evidence.reference
    evidence.reference = (points+20., normals)
    retained = evidence.reference
    for i in range(31, 40):
        evidence.observe(scan(x=.02*(i-31)), i*.1, 0.)
    assert evidence.local_healthy
    assert not evidence.healthy and not evidence.stationary
    assert evidence.reference is retained
    assert evidence.local_match.valid


def test_strong_old_reference_recovers_after_short_limit_without_resetting_pose():
    recovery = ReferenceRecovery([.24, 0.], reference_max_gap=10.)
    anchor = np.array([5., 2., .3])
    recovery.accept(scan(), 10., [0., 0., 0.], anchor, np.diag([.5, .6, .08]))
    for i in range(5):
        dx = .1+i*.01
        recovered = recovery.check(scan(x=dx), 13.+i*.1, [dx, 0., 0.], False, .015)
        if i < 4:
            assert not recovered
    assert recovered and recovery.method == 'reference'
    assert np.allclose(recovery.recovered_pose, compose_pose(anchor, [.14, 0., 0.]), atol=.006)
    assert np.linalg.eigvalsh(recovery.recovered_covariance-np.diag([.5, .6, .08])).min() >= -1e-9
    assert not recovery.check(scan(x=.14), 21., [.14, 0., 0.], False, .015)
    assert 'expired' in recovery.reason


def test_stale_stationary_flag_cannot_hide_missing_accumulated_motion():
    recovery = ReferenceRecovery([.24, 0.], reference_max_gap=10.)
    recovery.accept(scan(), 10., [0., 0., 0.])
    for i in range(6):
        ready = recovery.check(scan(x=.1), 10.5+i*.1, [0., 0., 0.], True, .015)
        if i < 4:
            assert not ready
    assert ready and np.allclose(recovery.recovered_pose, [.1, 0., 0.], atol=.006)


def failed_match(*args, **kwargs):
    return Match(np.zeros(3), np.eye(3), 0., 1., 0., False, 'low_overlap')


def bridge_sample(stamp):
    return inertial(stamp, [5.+.2*(stamp-10.), 2., .3])


def attempt_bridge(recovery, stamp, **overrides):
    current, previous = bridge_sample(stamp), bridge_sample(stamp-.1)
    observed = Match(relative_pose(previous['pose'], current['pose']), np.eye(3)*.001, .95, .01, .3, True, 'ok')
    options = dict(inertial=current, local_match=observed, local_inertial=previous,
                   local_stamp=stamp-.1, local_healthy=True)
    options.update(overrides)
    return recovery.check(scan(), stamp, [0., 0., 0.], False, .015, **options)


def test_inertial_bridge_retains_displacement_and_covariance(monkeypatch):
    monkeypatch.setattr('kiwi_slam.reference_recovery.align', failed_match)
    recovery = ReferenceRecovery([.24, 0.], reference_max_gap=10.)
    recovery.accept(scan(), 10., [0., 0., 0.], [5., 2., .3])
    assert not attempt_bridge(recovery, 10.5)
    assert not attempt_bridge(recovery, 10.6)
    assert attempt_bridge(recovery, 10.7)
    assert recovery.method == 'inertial_bridge'
    assert np.allclose(recovery.recovered_pose, [5.14, 2., .3])
    assert np.array_equal(recovery.recovered_covariance, np.diag([.5, .6, .08]))
    assert recovery.reference[1] == 10.


def test_inertial_bridge_rejects_invalid_covariance_time_missing_motion_and_deadline(monkeypatch):
    monkeypatch.setattr('kiwi_slam.reference_recovery.align', failed_match)
    recovery = ReferenceRecovery([.24, 0.], reference_max_gap=10.)
    recovery.accept(scan(), 10., [0., 0., 0.], [5., 2., .3])
    for stamp in (10.5, 10.6, 10.7):
        assert not attempt_bridge(recovery, stamp, inertial=None)
        assert not attempt_bridge(recovery, stamp, local_healthy=False)
        assert not attempt_bridge(recovery, stamp, inertial=inertial(stamp, [5., 2., .3], np.diag([4.1, .5, .08])))
        assert not attempt_bridge(recovery, stamp, inertial=inertial(stamp-.5, [5., 2., .3]))
        disagreement = bridge_sample(stamp-.1)
        disagreement['pose'] = disagreement['pose']-np.array([.3, 0., 0.])
        assert not attempt_bridge(recovery, stamp, local_inertial=disagreement)
        assert recovery.reason == 'inertial_scan_disagreement'
    assert not attempt_bridge(recovery, 12.1)
    assert recovery.reason == 'reference_mismatch'


def test_recovery_confirmation_must_be_continuous(monkeypatch):
    monkeypatch.setattr('kiwi_slam.reference_recovery.align', failed_match)
    recovery = ReferenceRecovery([.24, 0.])
    recovery.accept(scan(), 10., [0., 0., 0.], [5., 2., .3])
    assert not attempt_bridge(recovery, 10.5)
    assert not attempt_bridge(recovery, 10.6)
    assert not attempt_bridge(recovery, 11.)
    assert recovery.reason == 'recovery_motion_inconsistent'
    assert not attempt_bridge(recovery, 11.1)
    assert not attempt_bridge(recovery, 11.2)
    assert attempt_bridge(recovery, 11.3)


def test_covariance_floor_preserves_correlated_uncertainty_after_recovery():
    source = np.array([[.25, .02, 0.], [.02, .7, 0.], [0., 0., .01]])
    inherited = np.array([[.6, .05, .01], [.05, .5, 0.], [.01, 0., .08]])
    result = preserve_covariance_floor(source, inherited)
    assert np.linalg.eigvalsh(result-source).min() >= -1e-9
    assert np.linalg.eigvalsh(result-inherited).min() >= -1e-9
    assert np.allclose(result, result.T)


def test_adapter_can_recover_when_retained_reference_unhealthy_and_preserves_frame(monkeypatch):
    import copy
    from types import SimpleNamespace
    from unittest.mock import Mock
    from nav_msgs.msg import Odometry
    from kiwi_slam.odom_covariance import (OdometryCovarianceAdapter,
                                          StationaryFilterConfig, StationaryOdomFilter)

    monkeypatch.setattr('kiwi_slam.reference_recovery.align', failed_match)
    monkeypatch.setattr('kiwi_slam.odom_covariance.time.monotonic', lambda: 100.)
    evidence = ScanEvidence()
    for i in range(31):
        evidence.observe(scan(), 7.+i*.1, 0.)
    points, normals = evidence.reference
    evidence.reference = (points+20., normals)
    recovery = ReferenceRecovery([.24, 0.], reference_max_gap=10.)
    recovery.accept(scan(), 10., [0., 0., 0.], [5., 2., 0.])
    stationary_filter = StationaryOdomFilter(StationaryFilterConfig(2., .011, .75, .2, .02, .01, .012, .25))
    stationary_filter._anchor_offset([0., 0., 0.], [5., 2., 0.])
    node = SimpleNamespace(
        calibrated=True, calibrated_at=100., quality_origin=True, quality_pub=Mock(),
        evidence=evidence, needs_recovery=True, tracking_lost=False, recovery=recovery,
        last_output=(10., (5., 2., 0.)), stationary_filter=stationary_filter,
        recovery_covariance_floor=np.zeros((3, 3)), get_logger=Mock(return_value=Mock()),
        scan_cache={}, pending_odometry={}, last_match_pose=None, lidar_xy=[.24, 0.],
        variances=dict(pose_xy_variance=.25, pose_yaw_variance=.01, twist_vx_variance=.04,
                       twist_vy_variance=.01, twist_wz_variance=.04),
        publisher=Mock(), zero_velocity_pub=Mock(), validated_pub=Mock())
    node._inertial_at = lambda stamp: (None if stamp is None else
        inertial(stamp, [5.+.2*(stamp-10.), 2., 0.]))
    for i in range(1, 9):
        stamp = 10.+i*.1
        current = scan(x=.2*(stamp-10.))
        evidence.observe(current, stamp, 0.)
        odom = Odometry()
        odom.header.stamp.sec = 10
        odom.header.stamp.nanosec = i*100000000
        odom.pose.pose.orientation.w = 1.
        odom.twist.twist.linear.x = 99.
        node.scan_cache[stamp] = (current, copy.copy(evidence), 100.)
        OdometryCovarianceAdapter._quality_odometry(node, odom)
        if node.publisher.publish.called:
            break
    assert node.publisher.publish.call_count == 1
    result = node.publisher.publish.call_args.args[0]
    assert result.pose.pose.position.x > 5.
    assert math.isclose(result.pose.pose.position.x, 5.+.2*(stamp-10.))
    assert math.isclose(result.pose.pose.position.y, 2.)
    assert math.isclose(result.twist.twist.linear.x, .2, abs_tol=.001)
    assert result.pose.covariance[0] >= .5-1e-9 and result.pose.covariance[7] >= .6-1e-9
    assert not node.needs_recovery and not node.tracking_lost
    assert node.validated_pub.publish.call_count == 1
    assert node.zero_velocity_pub.publish.call_count == 0


def test_corridor_partial_velocity_preserves_weak_direction_uncertainty():
    from kiwi_slam.laser_quality import PartialVelocityObservation, align
    angle = .5
    observer = PartialVelocityObservation([.24, 0.])
    for i in range(1, 6):
        previous = scan(x=.01*(i-1), y=.02*(i-1), yaw=angle, corridor=True)
        current = scan(x=.01*i, y=.02*i, yaw=angle, corridor=True)
        match = align(cloud(previous), cloud(current))
        assert not match.valid and match.matched and match.observable_rank == 2
        assert match.overlap > .95 and match.yaw_observability > .95
        result = observer.observe(match, i*.1, (i-1)*.1)
    assert result is not None
    velocity, covariance = result
    weak = np.array([math.cos(angle), -math.sin(angle), 0.])
    strong = np.array([math.sin(angle), math.cos(angle), 0.])
    assert weak@covariance@weak > 1e5
    assert 0 < strong@covariance@strong < 1.
    assert abs(strong@velocity-.2) < .02
    assert np.linalg.eigvalsh(covariance).min() > 0
    assert abs(covariance[0, 1]) > 1e4


def test_partial_velocity_rejects_no_information_and_large_moving_foreground():
    from kiwi_slam.laser_quality import PartialVelocityObservation, align
    observer = PartialVelocityObservation([.24, 0.])
    empty = scan(); empty.ranges = [math.inf]*1080
    assert observer.observe(align(cloud(empty), cloud(empty)), 1., .9) is None
    circle = scan(); circle.ranges = [2.]*1080
    rotation_unobservable = align(cloud(circle), cloud(circle))
    assert rotation_unobservable.yaw_observability < .8
    assert observer.observe(rotation_unobservable, 1.1, 1.) is None
    foreground = scan(corridor=True)
    foreground.ranges = list(foreground.ranges[:250])+[.8]*550+list(foreground.ranges[800:])
    changed = align(cloud(scan(corridor=True)), cloud(foreground))
    for i in range(5):
        assert observer.observe(changed, 2.+i*.1, 1.9+i*.1) is None
    assert observer.count == 0


def test_small_passing_foreground_does_not_create_partial_self_motion():
    from kiwi_slam.laser_quality import PartialVelocityObservation, align
    observer = PartialVelocityObservation([.24, 0.])
    match = align(cloud(scan(corridor=True)), cloud(scan(corridor=True, person=True)))
    for i in range(5):
        result = observer.observe(match, 2.+i*.1, 1.9+i*.1)
    if result is not None:
        velocity, covariance = result
        assert abs(velocity[1]) < .02 and abs(velocity[2]) < .02
        assert covariance[0, 0] > 1e5


def adapter_fixture(monkeypatch, partial=True):
    from types import SimpleNamespace, MethodType
    from unittest.mock import Mock
    from kiwi_slam.laser_quality import PartialVelocityObservation
    from kiwi_slam.odom_covariance import (OdometryCovarianceAdapter,
                                          StationaryFilterConfig, StationaryOdomFilter)
    monkeypatch.setattr('kiwi_slam.odom_covariance.time.monotonic', lambda: 100.)
    evidence = ScanEvidence()
    for i in range(31): evidence.observe(scan(), 7.+i*.1, 0.)
    recovery = ReferenceRecovery([.24, 0.], reference_max_gap=10.)
    recovery.accept(scan(), 9.9, [0., 0., 0.], [0., 0., 0.])
    stationary = StationaryOdomFilter(StationaryFilterConfig(2., .011, .75, .2, .02, .01, .012, .25))
    stationary.held_pose = (0., 0., 0.)
    node = SimpleNamespace(
        calibrated=True, calibrated_at=100., quality_origin=True, quality_pub=Mock(),
        evidence=evidence, needs_recovery=False, tracking_lost=False, recovery=recovery,
        last_output=(9.9, (0., 0., 0.)), stationary_filter=stationary, held_reference=None,
        recovery_covariance_floor=np.zeros((3, 3)), get_logger=Mock(return_value=Mock()),
        scan_cache={}, pending_odometry={}, last_match_pose=None, lidar_xy=[.24, 0.],
        variances=dict(pose_xy_variance=.25, pose_yaw_variance=.01, twist_vx_variance=.04,
                       twist_vy_variance=.01, twist_wz_variance=.04),
        publisher=Mock(), zero_velocity_pub=Mock(), validated_pub=Mock(),
        partial_enabled=partial, partial=PartialVelocityObservation([.24, 0.]),
        partial_pub=Mock(), partial_ready_pub=Mock(), last_partial_at=None, base_frame='base_footprint')
    node._inertial_at = lambda stamp: None
    node._publish_partial_observation = MethodType(OdometryCovarianceAdapter._publish_partial_observation, node)
    node._clear_partial_observation = MethodType(OdometryCovarianceAdapter._clear_partial_observation, node)
    return node


def feed_adapter(node, stamp, x, message_scan=None, observe=True):
    import copy
    from nav_msgs.msg import Odometry
    from kiwi_slam.odom_covariance import OdometryCovarianceAdapter
    current = scan(x=x) if message_scan is None else message_scan
    if observe: node.evidence.observe(current, stamp, 0.)
    message = Odometry()
    message.header.frame_id, message.child_frame_id = 'odom', 'base_footprint'
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
    stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
    message.pose.pose.position.x = x
    message.pose.pose.orientation.w = 1.
    node.scan_cache[stamp] = (current, copy.copy(node.evidence), 100.)
    OdometryCovarianceAdapter._quality_odometry(node, message)


def test_normal_start_stays_healthy_and_release_preserves_confirmed_displacement(monkeypatch):
    node = adapter_fixture(monkeypatch)
    feed_adapter(node, 10., 0., observe=False)
    for i in range(1, 9):
        feed_adapter(node, 10.+i*.1, .03*i)
        assert node.evidence.healthy
        assert not node.needs_recovery and not node.tracking_lost
    assert node.publisher.publish.call_count == 9
    assert node.partial_pub.publish.call_count == 0
    assert abs(node.last_output[1][0]-.24) < .006
    assert all(call.args[0].data for call in node.quality_pub.publish.call_args_list)


def test_recovery_rechecks_future_cached_scans_before_delayed_odometry(monkeypatch):
    import copy
    from nav_msgs.msg import Odometry
    from kiwi_slam.odom_covariance import OdometryCovarianceAdapter

    node = adapter_fixture(monkeypatch)
    feed_adapter(node, 10., 0., observe=False)
    points, normals = node.evidence.reference
    node.evidence.reference = (points+20., normals)
    node.needs_recovery = True
    # 模拟scan回调比RF2O先到：所有未来快照都带着旧参考失效状态。
    for i in range(1, 9):
        stamp = 10.+i*.1
        current = scan(x=.02*i)
        node.evidence.observe(current, stamp, 0.)
        node.scan_cache[stamp] = (current, copy.copy(node.evidence), 100.)
        assert not node.evidence.healthy
    for i in range(1, 9):
        stamp = 10.+i*.1
        message = Odometry()
        message.header.frame_id, message.child_frame_id = 'odom', 'base_footprint'
        message.header.stamp.sec = 10
        message.header.stamp.nanosec = i*100000000
        message.pose.pose.position.x = .02*i
        message.pose.pose.orientation.w = 1.
        OdometryCovarianceAdapter._quality_odometry(node, message)
        if i >= 3:
            assert not node.needs_recovery and node.evidence.healthy
            assert not node.evidence.time_discontinuous
            assert abs(node.last_output[0]-stamp) < 1e-8
            assert abs(node.last_output[1][0]-.02*i) < .006
            assert all(cached.healthy for key, (_, cached, _) in node.scan_cache.items()
                       if key > stamp)
    assert node.publisher.publish.call_count == 7
    assert node.validated_pub.publish.call_count == 7
    assert node.recovery.reference[1] == 10.8


def test_recovery_scan_replay_does_not_promote_new_invalid_scan(monkeypatch):
    import copy
    from kiwi_slam.odom_covariance import OdometryCovarianceAdapter

    node = adapter_fixture(monkeypatch)
    node.evidence.accept_recovered_reference(scan(), 10.)
    invalid = scan(); invalid.ranges = [math.inf]*1080
    node.scan_cache[10.1] = (scan(x=.02), copy.copy(node.evidence), 100.)
    node.scan_cache[10.2] = (invalid, copy.copy(node.evidence), 100.)
    OdometryCovarianceAdapter._replay_scans_after_recovery(node, 10.)
    assert node.scan_cache[10.1][1].healthy
    assert not node.scan_cache[10.2][1].healthy
    assert not node.evidence.healthy


def test_partial_velocity_never_refreshes_full_anchor_or_mapping(monkeypatch):
    node = adapter_fixture(monkeypatch)
    feed_adapter(node, 10., 0., observe=False)
    previous_reference, previous_output = node.recovery.reference, node.last_output
    full_count, mapping_count = node.publisher.publish.call_count, node.validated_pub.publish.call_count
    for i in range(1, 31):
        feed_adapter(node, 10.+i*.1, .01*i, scan(x=.01*i, corridor=True))
    assert node.partial_pub.publish.call_count > 0
    assert node.publisher.publish.call_count == full_count
    assert node.validated_pub.publish.call_count == mapping_count
    assert node.recovery.reference is previous_reference and node.last_output == previous_output
    partial = node.partial_pub.publish.call_args.args[0]
    assert partial.child_frame_id == 'base_footprint'
    assert partial.pose.covariance[0] == 1e6 and partial.twist.covariance[0] > 1e5
    assert not node.quality_pub.publish.call_args.args[0].data
    assert node.partial_ready_pub.publish.call_args.args[0].data
    # 超过 INS 的 2s 完整观测预算后，部分速度仍不延长 full anchor 时间。
    assert node.recovery.reference[1] == 10.
    for i in range(31, 37):
        feed_adapter(node, 10.+i*.1, .01*i)
    assert node.publisher.publish.call_count > full_count
    assert abs(node.last_output[1][0]-.36) < .006
    assert node.validated_pub.publish.call_count > mapping_count


def test_transient_geometry_noise_is_not_promoted_to_permanent_origin_uncertainty(monkeypatch):
    node = adapter_fixture(monkeypatch)
    node.evidence.last_match.covariance = np.diag([10., .2, .001])
    feed_adapter(node, 10., 0., observe=False)
    assert node.publisher.publish.call_args.args[0].pose.covariance[0] > 10.
    assert np.array_equal(node.recovery.reference_covariance, np.zeros((3, 3)))
    assert np.array_equal(node.recovery_covariance_floor, np.zeros((3, 3)))


def test_inertial_recovery_retains_large_baseline_but_rejects_new_growth():
    recovery = ReferenceRecovery([.24, 0.])
    baseline = np.diag([12.5, .5, .2])
    recovery.accept(scan(), 10., [0., 0., 0.], covariance=np.zeros((3, 3)), baseline_covariance=baseline)
    assert recovery._valid_inertial(inertial(10.5, [.1, 0., 0.], baseline), 10.5)
    assert recovery._valid_inertial(inertial(10.5, [.1, 0., 0.], baseline+np.diag([.04, .04, .02])), 10.5)
    assert not recovery._valid_inertial(inertial(10.5, [.1, 0., 0.], baseline+np.diag([.091, .01, .02])), 10.5)
    assert not recovery._valid_inertial(inertial(10.5, [.1, 0., 0.], baseline+np.diag([.01, .01, .051])), 10.5)
