"""保护层在掉激光/异步恢复期间的行为回归。"""
from collections import deque
import math
from types import MethodType, SimpleNamespace
from unittest.mock import Mock
import numpy as np
import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from kiwi_slam.estimation_guard import EstimationGuard, inertial_covariance_valid


def test_guard_heartbeat_readers_do_not_replay_history_but_status_remains_latched(monkeypatch):
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    readers, writers = {}, {}
    monkeypatch.setattr(Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(Node, 'declare_parameter', lambda self, name, default:
                        SimpleNamespace(value=True if name == 'inertial_enabled' else default))
    def subscription(self, message_type, topic, callback, qos):
        readers[topic] = qos
        return Mock()
    def publisher(self, message_type, topic, qos):
        writers[topic] = qos
        return Mock()
    monkeypatch.setattr(Node, 'create_subscription', subscription)
    monkeypatch.setattr(Node, 'create_publisher', publisher)
    monkeypatch.setattr(Node, 'create_client', Mock())
    monkeypatch.setattr(Node, 'create_timer', Mock())
    monkeypatch.setattr('kiwi_slam.estimation_guard.TransformBroadcaster', Mock())
    monkeypatch.setattr(EstimationGuard, 'tick', Mock())
    EstimationGuard()
    for topic in ('/chassis/imu/data_ready', '/chassis/odom/laser_ready',
                  '/chassis/inertial/valid'):
        qos = readers[topic]
        assert qos.depth == 10
        assert qos.reliability == ReliabilityPolicy.RELIABLE
        assert qos.durability == DurabilityPolicy.VOLATILE
    for topic in ('/chassis/estimation/valid', '/chassis/estimation/state'):
        assert writers[topic].durability == DurabilityPolicy.TRANSIENT_LOCAL
        assert writers[topic].depth == 1


def odom(t, x=0.):
    m = Odometry()
    m.header.frame_id, m.child_frame_id = 'odom', 'base_footprint'
    m.header.stamp.sec = int(t)
    m.header.stamp.nanosec = round((t-int(t))*1e9)
    m.pose.pose.orientation.w = 1.
    m.pose.pose.position.x = x
    m.pose.covariance[0] = m.pose.covariance[7] = .25
    m.pose.covariance[35] = .01
    return m


@pytest.fixture
def guard(monkeypatch):
    time = [100.]
    monkeypatch.setattr('kiwi_slam.estimation_guard.time.monotonic', lambda: time[0])
    n = SimpleNamespace(
        max_age=.2, max_source_age=.35, recovery_duration=.3,
        max_error=.25, max_yaw_error=.15, inertial_enabled=True, max_coast=2.,
        max_inertial_xy=1., max_inertial_yaw=.15, max_inertial_growth=.09, max_inertial_yaw_growth=.05,
        trusted_xy_variance=.25, trusted_yaw_variance=.01,
        filtered_history=deque(maxlen=120), laser_history=deque(maxlen=50),
        gyro_history=deque(maxlen=200), last_source_message={},
        prediction_acceleration_bound=1.5, prediction_yaw_acceleration_bound=2.,
        inconsistent_since=None, handoff_started=None, last_output=None,
        output_source=None, output_offset=[0.,0.,0.], correction_speed=.05, correction_yaw_rate=.10,
        odom_frame='odom', base_frame='base_footprint', ever_valid=True,
        flags={}, laser=None, gyro=None, inertial=None,
        laser_at=None, gyro_at=None, inertial_at=None,
        last_trusted_laser_stamp=100., health_reason='',
        observation_reason='', pause_reason=None, filtered_epoch_stamp=None,
        filtered_reject_reason='waiting_filtered_output',
        mode='VALID', valid=True, inertial_output=False,
        future=None, future_operation=None, request_started=None,
        recovery_since=None, enabled_at=None, last_forwarded=(99.9,99.9),
        last_state=None, publisher=Mock(), tf=Mock(), valid_pub=Mock(), state_pub=Mock(),
        toggle=Mock(), set_pose=Mock(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(time[0]*1e9))),
        get_logger=Mock(return_value=Mock()))
    for name, method in EstimationGuard.__dict__.items():
        if callable(method) and not name.startswith('__'):
            setattr(n, name, MethodType(method,n))
    n.toggle.service_is_ready.return_value=False
    n.set_pose.service_is_ready.return_value=False
    def samples(t, laser=False):
        time[0]=t
        n.gyro=Imu();n.gyro.header=odom(t).header;n.gyro.header.frame_id='base_footprint'
        n.gyro_at=t
        n.on_gyro(n.gyro)
        n.flags={'imu':(True,t),'laser':(laser,t),'inertial':(True,t)}
        n.inertial,n.inertial_at=odom(t,.2*(t-100)),t
        if laser:
            n.laser,n.laser_at=odom(t,.2*(t-100)),t
            n.laser_history.append(n.laser)
    return n,time,samples


def test_laser_failure_does_not_disable_ekf_during_valid_inertial_bridge(guard):
    n,_,samples=guard
    for i in range(1,15):
        samples(100+i*.1);n.tick()
        assert n.mode=='INERTIAL' and n.valid
    n.toggle.call_async.assert_not_called()
    assert n.publisher.publish.call_count==14
    assert n.publisher.publish.call_args.args[0].pose.pose.position.x > .27


def test_exhausted_inertial_window_is_invalid_and_pauses_instead_of_infinite_integration(guard):
    n,_,samples=guard;samples(102.01);n.tick()
    assert n.mode=='PAUSING' and not n.valid
    n.publisher.publish.assert_not_called()


def test_late_repeated_candidate_cannot_refresh_its_timestamp(guard):
    n,time,samples=guard;samples(100.1);n.tick()
    assert n.valid
    samples(100.5);n.inertial=odom(100.1);n.tick()
    assert not n.valid


def test_short_recovery_keeps_ekf_state_and_velocity_without_set_pose(guard):
    n,time,samples=guard
    samples(100.,True);n.on_filtered(odom(100.))
    samples(100.2);n.tick()
    samples(100.4,True);n.tick();n.on_filtered(odom(100.4,.08))
    samples(100.8,True);n.tick();n.on_filtered(odom(100.8,.16))
    # 停止INS抢时间戳后下一EKF帧接管，不调用reset/enable。
    samples(100.84,True);n.on_filtered(odom(100.84,.168))
    assert n.mode=='VALID' and not n.inertial_output
    n.set_pose.call_async.assert_not_called()
    n.toggle.call_async.assert_not_called()


def test_paused_startup_does_not_bootstrap_from_inertial_alone(guard):
    n,_,samples=guard;n.mode='PAUSED';n.ever_valid=False
    samples(100.1);n.tick()
    assert not n.valid and n.mode=='PAUSED'


def test_service_completes_after_sensor_loss_without_enabling_bad_output(guard):
    n,_,samples=guard;samples(103.)
    future=Mock();future.done.return_value=True;future.result.return_value=SimpleNamespace()
    n.future=future;n.future_operation='reset';n.mode='RESETTING'
    n.tick()
    assert n.mode=='PAUSING' and not n.valid


def test_covariance_limits_reject_nonfinite_or_zero_confidence():
    m=odom(1.)
    assert inertial_covariance_valid(m,1.,.15)
    m.pose.covariance[0]=0.
    assert not inertial_covariance_valid(m,1.,.15)
    m.pose.covariance[0]=float('nan')
    assert not inertial_covariance_valid(m,1.,.15)


def test_missing_ekf_output_does_not_wait_forever_after_enable(guard):
    n,_,samples=guard
    samples(104., True)
    n.mode='WARMING';n.enabled_at=100.;n.valid=False
    n.tick()
    assert n.mode=='PAUSING' and not n.valid
    assert n.health_reason.startswith('ekf_output_recovery_timeout:')


@pytest.mark.parametrize('operation,status,expected', [
    ('pause', False, 'PAUSED'), ('enable', False, 'WARMING')])
def test_idempotent_humble_toggle_response_is_not_a_failure(guard, operation, status, expected):
    n,time,_=guard
    future=Mock();future.done.return_value=True;future.result.return_value=SimpleNamespace(status=status)
    n.future=future;n.future_operation=operation;n.request_started=time[0]
    n.finish_request(time[0])
    assert n.mode==expected and n.future is None


def test_aligned_comparison_does_not_reset_perfect_delayed_turn():
    from kiwi_slam.estimation_guard import aligned_consistency
    def rotating(t):
        m=odom(t)
        m.pose.pose.orientation.z=math.sin(.95*(t-100)/2)
        m.pose.pose.orientation.w=math.cos(.95*(t-100)/2)
        m.twist.twist.angular.z=.95
        return m
    filtered=deque(rotating(t) for t in [100.,100.1,100.2,100.3])
    laser=deque(rotating(t) for t in [100.,100.0833,100.13])
    assert aligned_consistency(filtered,laser,.25,.15) is True
    laser[-1].pose.pose.orientation.z=math.sin(.7/2)
    laser[-1].pose.pose.orientation.w=math.cos(.7/2)
    assert aligned_consistency(filtered,laser,.25,.15) is False


def test_ins_cannot_starve_delayed_ekf_during_handoff(guard):
    n,time,samples=guard
    samples(100.,True);n.on_filtered(odom(100.))
    samples(100.1);n.tick()
    for i in range(2,18):
        t=100+i*.1
        samples(t,True)
        n.tick()
        m=odom(t-.03,.2*(t-.03-100))
        n.on_filtered(m)
    assert n.mode=='VALID' and n.valid and not n.inertial_output
    n.set_pose.call_async.assert_not_called()


def test_switching_sources_does_not_create_pose_jump(guard):
    n,_,_=guard
    first=odom(100.,0.);first.twist.twist.linear.x=.12
    assert n.forward(first,'filtered')
    candidate=odom(100.03,.0736);candidate.twist.twist.linear.x=.12
    assert n.forward(candidate,'inertial')
    output=n.publisher.publish.call_args.args[0]
    assert output.pose.pose.position.x==pytest.approx(.0036)
    assert output.twist.twist.linear.x==pytest.approx(.12)
    assert output.pose.covariance[0] >= candidate.pose.covariance[0]+.07**2-1e-10
    assert candidate.pose.pose.position.x==.0736  # 不改候选原始数据。


def test_handoff_correction_is_bounded_and_accounts_for_twist(guard):
    n,_,_=guard
    first=odom(100.,0.);first.twist.twist.linear.x=.12;n.forward(first,'inertial')
    second=odom(100.03,.0736);second.twist.twist.linear.x=.12;n.forward(second,'filtered')
    third=odom(100.06,.0772);third.twist.twist.linear.x=.12;n.forward(third,'filtered')
    output=n.publisher.publish.call_args.args[0]
    assert output.pose.pose.position.x==pytest.approx(.0087)
    assert output.twist.twist.linear.x==pytest.approx(.17)


def test_large_inherited_anchor_uncertainty_does_not_disable_bounded_ins(guard):
    n,_,samples=guard
    n.trusted_xy_variance=12.516578851
    samples(100.1)
    n.inertial.pose.covariance[0]=12.52
    n.inertial.pose.covariance[7]=12.52
    assert n.inertial_usable()
    n.inertial.pose.covariance[0]=12.7
    assert not n.inertial_usable()


def test_handoff_rejects_unbounded_disagreement(guard):
    n,_,_=guard
    n.forward(odom(100.,0.),'filtered')
    assert not n.forward(odom(100.03,1.),'inertial')
    assert not n.valid and n.mode=='PAUSING'


@pytest.mark.parametrize('bad',['pose_jump','velocity_nan','covariance_nan','covariance_indefinite','quaternion','speed'])
def test_latest_bad_ekf_frame_is_rejected_even_when_aligned_old_frame_is_good(guard,bad):
    n,_,samples=guard
    samples(100.,True);n.on_filtered(odom(100.))
    samples(100.1,True)
    n.laser=odom(100.);n.laser_history=deque([n.laser]);n.laser_at=100.1
    message=odom(100.1,.02)
    if bad=='pose_jump':message.pose.pose.position.x=10.
    if bad=='velocity_nan':message.twist.twist.linear.x=math.nan
    if bad=='covariance_nan':message.pose.covariance[0]=math.nan
    if bad=='covariance_indefinite':message.pose.covariance[1]=message.pose.covariance[6]=2.
    if bad=='quaternion':message.pose.pose.orientation.w=2.
    if bad=='speed':message.twist.twist.linear.x=10.
    before=n.publisher.publish.call_count
    n.on_filtered(message)
    assert n.publisher.publish.call_count==before
    assert not n.valid and n.mode=='PAUSING'


def test_same_source_cannot_publish_a_physical_pose_jump(guard):
    n,_,_=guard
    assert n.forward(odom(100.,0.),'filtered')
    assert not n.forward(odom(100.03,.2),'filtered')
    assert n.publisher.publish.call_count==1


def test_twist_covariance_rotates_with_handoff_yaw_offset(guard):
    n,_,_=guard
    n.forward(odom(100.),'inertial')
    message=odom(100.03)
    message.pose.pose.orientation.z=math.sin(-.1/2);message.pose.pose.orientation.w=math.cos(-.1/2)
    message.twist.twist.linear.x=.2
    matrix=np.eye(6)*.01;matrix[1,1]=1.;matrix[1,5]=matrix[5,1]=.02
    message.twist.covariance=matrix.ravel().tolist()
    assert n.forward(message,'filtered')
    output=n.publisher.publish.call_args.args[0]
    rotation=np.eye(6);c,s=math.cos(.1),math.sin(.1);rotation[:2,:2]=[[c,s],[-s,c]]
    assert np.allclose(np.array(output.twist.covariance).reshape(6,6),rotation@matrix@rotation.T)
    assert output.twist.twist.linear.y==pytest.approx(-.2*math.sin(.1))


def test_interpolated_message_has_the_requested_timestamp():
    from kiwi_slam.estimation_guard import interpolate_odometry,stamp_seconds
    result=interpolate_odometry([odom(100.),odom(100.1,.02)],100.05)
    assert stamp_seconds(result)==pytest.approx(100.05)
    assert result.pose.pose.position.x==pytest.approx(.01)


def test_new_laser_arriving_twenty_ms_before_imu_does_not_pulse_invalid(guard):
    n,_,samples=guard
    n.mode='INERTIAL';n.inertial_output=True
    samples(100.1,True)
    n.inertial=odom(100.08,.016)
    n.tick()
    assert n.last_trusted_laser_stamp==pytest.approx(100.1)
    assert n.inertial_usable() and n.valid and n.mode=='INERTIAL'
    n.inertial=odom(99.89)
    assert not n.inertial_usable()


def test_persistent_quarter_second_delayed_turning_ekf_recovers_without_stopping_ins(guard):
    n,clock,samples=guard
    speed,rate=.2,.6
    def moving(stamp):
        elapsed=stamp-100.
        message=odom(stamp,speed/rate*math.sin(rate*elapsed))
        message.pose.pose.position.y=speed/rate*(1.-math.cos(rate*elapsed))
        message.pose.pose.orientation.z=math.sin(rate*elapsed/2)
        message.pose.pose.orientation.w=math.cos(rate*elapsed/2)
        message.twist.twist.linear.x=speed;message.twist.twist.angular.z=rate
        return message
    n.laser_history=deque(moving(99.5+i*.05) for i in range(11))
    n.filtered_history=deque([moving(99.5)])
    n.gyro_history=deque((99.5+i*.01,rate,.001) for i in range(51))
    n.mode='INERTIAL';n.inertial_output=True;n.recovery_since=99.
    assert n.forward(moving(100.),'inertial')
    for i in range(41):
        stamp=100.+i*.05;samples(stamp,True)
        n.laser=moving(stamp);n.laser_history[-1]=n.laser;n.inertial=moving(stamp)
        n.gyro.angular_velocity.z=rate;n.gyro_history[-1]=(stamp,rate,.001)
        n.tick();n.on_filtered(moving(stamp-.25))
        assert n.valid and n.mode in ('INERTIAL','VALID')
        assert stamp-n.last_forwarded[0] < .1
    assert n.mode=='VALID' and not n.inertial_output
    n.toggle.call_async.assert_not_called();n.set_pose.call_async.assert_not_called()
    output=n.publisher.publish.call_args.args[0];expected=moving(102.)
    assert math.hypot(output.pose.pose.position.x-expected.pose.pose.position.x,
                      output.pose.pose.position.y-expected.pose.pose.position.y) < .001
    assert output.pose.covariance[0] > .25


def test_delay_prediction_integrates_changing_gyro_and_adds_uncertainty():
    from kiwi_slam.estimation_guard import predict_odometry,stamp_seconds,yaw
    message=odom(100.);message.twist.twist.linear.x=.2;message.twist.twist.angular.z=.4
    gyro=[(100.+i*.01,.4+.5*i*.01,.001) for i in range(31)]
    predicted=predict_odometry(message,100.3,gyro)
    time=np.linspace(0.,.3,10001);angle=.4*time+.25*time*time
    expected=[np.trapz(.2*np.cos(angle),time),np.trapz(.2*np.sin(angle),time)]
    assert stamp_seconds(predicted)==pytest.approx(100.3)
    assert predicted.pose.pose.position.x==pytest.approx(expected[0],abs=3e-6)
    assert predicted.pose.pose.position.y==pytest.approx(expected[1],abs=3e-6)
    assert yaw(predicted)==pytest.approx(.1425,abs=1e-10)
    assert predicted.twist.twist.angular.z==pytest.approx(.55)
    assert predicted.pose.covariance[0] > message.pose.covariance[0]
    assert np.linalg.eigvalsh(np.array(predicted.pose.covariance).reshape(6,6)).min() >= -1e-10
    assert predict_odometry(message,100.36,gyro) is None
    assert stamp_seconds(message)==pytest.approx(100.)


def test_smaller_covariance_from_new_laser_does_not_reject_older_valid_ins(guard):
    n,_,samples=guard
    n.mode='INERTIAL';n.inertial_output=True
    samples(100.,True)
    n.laser.pose.covariance[0]=n.laser.pose.covariance[7]=12.5
    n.laser.pose.covariance[35]=.3
    n.trusted_xy_variance=12.5;n.trusted_yaw_variance=.3
    samples(100.1,True)
    n.inertial=odom(100.08)
    n.inertial.pose.covariance[0]=n.inertial.pose.covariance[7]=12.501
    n.inertial.pose.covariance[35]=.301
    n.tick()
    assert n.trusted_xy_variance==pytest.approx(.25)
    assert n.inertial_usable() and n.valid
    assert n.publisher.publish.call_count==1


@pytest.mark.parametrize('mode', ['PAUSING', 'PAUSED', 'RESETTING', 'ENABLING'])
@pytest.mark.parametrize('bad', ['frame', 'jump', 'nonfinite', 'ordinary'])
def test_queued_ekf_messages_cannot_change_suspended_filter_epoch(guard, mode, bad):
    n,_,samples=guard
    samples(100.,True)
    n.filtered_history.append(odom(99.95))
    n.mode=mode;n.valid=False
    message=odom(100.,.01)
    if bad=='frame':message.header.frame_id='wrong_frame'
    if bad=='jump':message.pose.pose.position.x=1.
    if bad=='nonfinite':message.twist.twist.linear.x=math.nan
    n.on_filtered(message)
    assert n.mode==mode
    assert not n.valid
    assert len(n.filtered_history)==1
    n.publisher.publish.assert_not_called()


def test_successful_set_pose_starts_new_filtered_epoch_without_zeroing_position(guard):
    n,clock,samples=guard
    samples(100.,True)
    n.on_filtered(odom(100.))
    n.mode='PAUSED';n.valid=False
    samples(100.05,True)
    n.laser.pose.pose.position.x=.5
    n.set_pose.service_is_ready.return_value=True
    future=Mock();future.done.return_value=True;future.result.return_value=SimpleNamespace()
    n.set_pose.call_async.return_value=future
    n.request_reset()
    assert n.set_pose.call_async.call_args.args[0].pose.pose.pose.position.x==.5
    clock[0]=100.08
    n.finish_request(clock[0])
    assert n.mode=='ENABLING'
    assert not n.filtered_history
    assert 'filtered' not in n.last_source_message
    assert n.last_forwarded[0]==pytest.approx(100.)  # 公共 TF 时间不能倒退。
    n.mode='WARMING';n.enabled_at=100.09
    samples(100.10,True)
    n.laser.pose.pose.position.x=.5
    queued=odom(100.07,10.)  # 请求后、确认前已排队的旧 epoch 消息。
    n.on_filtered(queued)
    assert n.mode=='WARMING' and not n.filtered_history
    n.on_filtered(odom(100.10,.5))
    assert n.mode=='WARMING' and len(n.filtered_history)==1
    samples(100.45,True);n.laser.pose.pose.position.x=.5
    n.on_filtered(odom(100.45,.5))
    assert n.mode=='VALID' and n.valid
    assert n.publisher.publish.call_args.args[0].pose.pose.position.x==pytest.approx(.5)


def test_pause_reason_survives_healthy_observations_at_next_tick(guard):
    n,_,samples=guard
    samples(100.,True);n.on_filtered(odom(100.))
    samples(100.05,True)
    message=odom(100.05);message.twist.twist.linear.x=10.
    n.on_filtered(message)
    assert n.mode=='PAUSING'
    reason=n.health_reason
    assert 'speed' in reason and '10' in reason
    n.tick()
    assert n.health_reason==reason
    assert reason in n.get_logger.return_value.info.call_args.args[0]


def test_output_watchdog_reports_why_fresh_sensor_input_cannot_publish(guard):
    n,_,samples=guard
    samples(100.5,True)
    n.last_forwarded=(100.,100.)
    n.tick()
    assert n.mode=='PAUSING' and not n.valid
    assert 'output_stale' in n.health_reason and '0.500' in n.health_reason
