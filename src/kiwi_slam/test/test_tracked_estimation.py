import math
import numpy as np
from sensor_msgs.msg import LaserScan
from kiwi_slam.laser_quality import ScanEvidence, StationaryAccelerationVeto, align, cloud
from kiwi_slam.imu_startup import StartupCalibration


def scan(x=0.,y=0.,yaw=0., corridor=False, person=False):
    result=LaserScan(); result.angle_min=-math.pi; result.angle_increment=2*math.pi/1080
    result.range_min=.15;result.range_max=12.
    angles=result.angle_min+np.arange(1080)*result.angle_increment+yaw
    dx,dy=np.cos(angles),np.sin(angles)
    bounds=(-100.,100.,-2.,2.) if corridor else (-3.,5.,-2.,4.)
    with np.errstate(divide='ignore'):
        tx=np.where(dx>0,(bounds[1]-x)/dx,(bounds[0]-x)/dx)
        ty=np.where(dy>0,(bounds[3]-y)/dy,(bounds[2]-y)/dy)
    ranges=np.minimum(tx,ty)
    if person:ranges[450:570]=.8
    ranges[ranges>=12]=np.inf
    result.ranges=ranges.tolist()
    return result


def test_background_alignment_rejects_foreground_and_preserves_real_translation_rotation():
    ref=cloud(scan())
    person=align(ref,cloud(scan(person=True)))
    assert person.valid and np.linalg.norm(person.pose)<.005
    for x,y,yaw in [(.10,0.,0.),(-.10,0.,0.),(0.,0.,.12),(0.,0.,-.12)]:
        m=align(ref,cloud(scan(x,y,yaw)))
        assert m.valid
        assert np.allclose(m.pose,[x,y,yaw],atol=.006)


def test_corridor_is_degraded_not_confident_stationary():
    e=ScanEvidence()
    for t in np.arange(0,5,.1):e.observe(scan(corridor=True),float(t),0.)
    assert not e.healthy and not e.stationary
    assert e.reason=='degenerate'


def test_person_passing_does_not_release_but_slow_forward_and_reverse_do():
    for direction in (-1.,1.):
        e=ScanEvidence()
        for t in np.arange(0,3,.1):e.observe(scan(),float(t),0.)
        assert e.stationary
        for t in np.arange(3,5,.1):
            e.observe(scan(person=True),float(t),0.)
            assert e.stationary and e.healthy
        released=False
        for i in range(1,60):
            e.observe(scan(x=direction*i*.004),5+i*.1,0.)
            released |= e.reason=='moving'
        assert released and not e.stationary


def test_rotation_releases_even_when_translation_is_zero():
    e=ScanEvidence()
    for i in range(31):e.observe(scan(),i*.1,0.)
    for i in range(1,21):e.observe(scan(yaw=i*.01),3+i*.1,.1)
    assert e.healthy and not e.stationary


def test_acceleration_veto_rejects_stationary_noise_and_one_sample_impact():
    covariance = np.eye(2)*.0025
    veto = StationaryAccelerationVeto()
    rng = np.random.default_rng(17)
    for i in range(301):
        stamp = i/30.
        assert not veto.push(stamp, rng.normal(0., .035, 2), covariance)
        assert not veto.active(stamp)
    veto = StationaryAccelerationVeto()
    for i in range(91):
        stamp = i/30.
        acceleration = [3., 0.] if i == 45 else [0., 0.]
        assert not veto.push(stamp, acceleration, covariance)
        assert not veto.active(stamp)


def test_continuous_acceleration_vetoes_zero_velocity_then_expires():
    veto = StationaryAccelerationVeto()
    for i in range(31):
        stamp = i/30.
        significant = veto.push(stamp, [.4, 0.] if 12 <= i <= 21 else [0., 0.],
                                np.eye(2)*.0025)
        if 15 <= i <= 21:
            assert significant and veto.active(stamp)
    assert veto.active(1.)
    for i in range(31, 46):
        stamp = i/30.
        assert not veto.push(stamp, [0., 0.], np.eye(2)*.0025)
    assert not veto.active(1.5)


def test_acceleration_veto_requires_calibrated_base_frame_and_fresh_samples(monkeypatch):
    from types import SimpleNamespace
    from unittest.mock import Mock
    from sensor_msgs.msg import Imu
    from kiwi_slam.odom_covariance import OdometryCovarianceAdapter

    clock = SimpleNamespace(nanoseconds=1000000000)
    node = SimpleNamespace(calibrated=True, base_frame='base_footprint',
        get_clock=lambda: SimpleNamespace(now=lambda: clock),
        acceleration_veto=StationaryAccelerationVeto(), acceleration_veto_enabled=True,
        acceleration_veto_received=None, stationary_pub=Mock())
    monkeypatch.setattr('kiwi_slam.odom_covariance.time.monotonic', lambda: 100.)
    message = Imu()
    message.header.frame_id = 'base_footprint'
    message.linear_acceleration.x = .4
    message.linear_acceleration_covariance = (np.eye(3)*.0025).ravel().tolist()
    for i in range(7):
        message.header.stamp.nanosec = i*33333333
        OdometryCovarianceAdapter._on_acceleration(node, message)
    assert not node.acceleration_veto.samples  # 全部超过新鲜度范围。
    clock.nanoseconds = 0
    for i in range(4):
        node.calibrated = False
        OdometryCovarianceAdapter._on_acceleration(node, message)
        node.calibrated = True
        message.header.frame_id = 'chassis_imu_link'
        OdometryCovarianceAdapter._on_acceleration(node, message)
    assert not node.acceleration_veto.samples and not node.stationary_pub.publish.called
    message.header.frame_id = 'base_footprint'
    for i in range(7):
        clock.nanoseconds = message.header.stamp.nanosec = i*33333333
        OdometryCovarianceAdapter._on_acceleration(node, message)
    evidence = SimpleNamespace(stationary=True, reason='stationary')
    assert not OdometryCovarianceAdapter._zero_velocity_allowed(node, evidence, .2)
    assert node.stationary_pub.publish.called
    assert all(not call.args[0].data for call in node.stationary_pub.publish.call_args_list)
    monkeypatch.setattr('kiwi_slam.odom_covariance.time.monotonic', lambda: 100.5)
    assert OdometryCovarianceAdapter._zero_velocity_allowed(node, evidence, .2)


def test_scan_loss_does_not_reanchor_and_falsely_recover():
    e=ScanEvidence()
    for i in range(31):e.observe(scan(),i*.1,0.)
    e.observe(scan(x=.4),6.,0.)
    for i in range(1,30):e.observe(scan(x=.4),6+i*.1,0.)
    assert not e.healthy and not e.stationary
    assert e.reason == 'scan_gap_requires_reference_validation'
    assert e.local_healthy  # 新扫描几何可恢复，但仍需外部核验原绝对参考。


def test_startup_estimates_residual_only_and_rejects_motion_or_gaps():
    c=StartupCalibration(5.);rng=np.random.default_rng(2);rest=np.array([-7.89,-.02,-5.91])
    for i in range(400):assert not c.push(i*.01,rest,.04,False)
    for i in range(400,951):c.push(i*.01,rest+rng.normal(0,.04,3),.002+rng.normal(0,.004),True)
    assert c.ready and abs(c.bias-.002)<.0005
    assert np.allclose(c.rest,rest,atol=.005)
    assert np.linalg.eigvalsh(c.covariance).min()>0
    assert not c.push(12.,rest,.002,True)
    assert c.failed and not c.ready


def test_moving_startup_cannot_accumulate_disjoint_stationary_windows():
    c=StartupCalibration(5.)
    for i in range(1000):
        c.push(i*.01,[0.,0.,9.81],.0,(i%400)<300)
    assert not c.ready


def test_invalid_initial_scan_does_not_poison_the_future_reference():
    e=ScanEvidence();empty=scan();empty.ranges=[math.inf]*1080
    e.observe(empty,0.,0.)
    assert not e.healthy
    for i in range(1,40):e.observe(scan(),i*.1,0.)
    assert e.healthy and e.stationary


def test_adapter_uses_sample_time_despite_callback_delay(monkeypatch):
    from collections import deque
    from types import SimpleNamespace
    from unittest.mock import Mock
    from kiwi_slam.odom_covariance import OdometryCovarianceAdapter

    node = SimpleNamespace(
        quality_enabled=True, scan_frame='lidar_link', quality_origin=True,
        evidence=ScanEvidence(), gyro_history=deque(), last_reason=None,
        scan_cache={}, pending_odometry={}, stationary_pub=Mock(), quality_pub=Mock(),
        get_logger=Mock(return_value=Mock()), last_scan_received=None)
    for i in range(40):
        # 在第 31 帧后积压 0.4 秒，但传感器采样仍为连续 10 Hz。
        stamp = 1000. + i * .1
        received = 10. + i * .1 + (.4 if i >= 31 else 0.)
        monkeypatch.setattr('kiwi_slam.odom_covariance.time.monotonic', lambda: received)
        node.last_gyro = received
        node.gyro_history = deque([(received, 0.)])
        message = scan()
        message.header.frame_id = 'lidar_link'
        message.header.stamp.sec = 1000+i//10
        message.header.stamp.nanosec = (i % 10)*100000000
        OdometryCovarianceAdapter._on_scan(node, message)
    assert node.evidence.healthy and node.evidence.stationary
    assert abs(node.evidence.last_time-stamp) < 1e-6
    assert node.last_scan_received == received


def test_short_imu_gap_keeps_calibration_but_requires_continuous_recovery():
    c=StartupCalibration(5.)
    rest=[0.,0.,9.81]
    for i in range(601): c.push(i*.01,rest,.002,True)
    bias,gravity=c.bias,c.rest.copy()
    assert c.calibrated and c.ready
    assert not c.push(6.34,rest,.01,False)
    assert c.calibrated and not c.ready and not c.failed
    for i in range(1,20):
        assert not c.push(6.34+i*.00001,rest,.01,False)
    for i in range(1,40): c.push(6.35+i*.01,rest,.01,False)
    assert c.ready and c.calibrated
    assert c.bias==bias and np.array_equal(c.rest,gravity)
    c.push(12.,rest,.01,False)
    assert c.failed and not c.calibrated


def test_reference_recovery_checks_motion_without_resetting_origin():
    from kiwi_slam.reference_recovery import ReferenceRecovery
    for direction in (-1.,1.):
        r=ReferenceRecovery([.24,0.])
        r.accept(scan(),10.,[5.,2.,0.])
        for i in range(5):
            x=direction*(.10+i*.01)
            ok=r.check(scan(x=x),10.5+i*.1,[5.+x,2.,0.],False,.015)
        assert ok and r.reference[1]==10.
        # 雷达看见位移，RF2O 却回到原点：不能伪装成恢复。
        assert not r.check(scan(x=.3),11.,[5.,2.,0.],False,.015)
        assert not r.check(scan(),13.,[5.,2.,0.],True,.015)
        assert 'expired' in r.reason


def test_guard_rejects_unbounded_filter_prediction():
    from kiwi_slam.estimation_guard import consistent_output
    from nav_msgs.msg import Odometry
    a,b=Odometry(),Odometry()
    a.pose.pose.orientation.w=b.pose.pose.orientation.w=1.
    assert consistent_output(a,b,.25,.15)
    a.pose.pose.position.x=152.
    assert not consistent_output(a,b,.25,.15)
    a.pose.pose.position.x=0.
    a.twist.twist.linear.x=6.
    assert not consistent_output(a,b,.25,.15)


def test_recovery_rotation_uses_lidar_lever_arm():
    from kiwi_slam.reference_recovery import ReferenceRecovery
    r=ReferenceRecovery([.24,0.])
    r.accept(scan(x=.24),10.,[0.,0.,0.])
    for i in range(5):
        angle=.10+i*.01
        ok=r.check(scan(x=.24*math.cos(angle),y=.24*math.sin(angle),yaw=angle),
                   10.5+i*.1,[0.,0.,angle],False,.015)
    assert ok
