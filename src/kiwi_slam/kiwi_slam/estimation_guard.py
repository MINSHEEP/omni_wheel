"""履带融合输出保护：有界激光/惯导续接、可信观测恢复，并独占发布 odom TF。"""
from collections import deque
from copy import deepcopy
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster
from robot_localization.srv import SetPose, ToggleFilterProcessing


def stamp_seconds(message):
    return message.header.stamp.sec+message.header.stamp.nanosec*1e-9


def yaw(message):
    q = message.pose.pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def set_stamp(message, stamp):
    nanos = int(round(stamp*1e9))
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(nanos, 1000000000)


def candidate_issue(message):
    """返回实际待发布帧的拒绝原因；不能用较旧插值帧替代检查。"""
    p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
    values = [p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.linear.x,v.linear.y,v.linear.z,
              v.angular.x,v.angular.y,v.angular.z,*message.pose.covariance,*message.twist.covariance]
    if not all(math.isfinite(x) for x in values):
        return 'nonfinite_pose_twist_or_covariance'
    norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
    if abs(norm-1.) > .01:
        return f'quaternion_norm_squared={norm:.6f}, expected=1+/-0.01'
    speed=math.hypot(v.linear.x,v.linear.y)
    if speed > .8:
        return f'linear_speed={speed:.6f}m/s, limit=0.8m/s'
    if abs(v.angular.z) > 1.8:
        return f'yaw_speed={v.angular.z:.6f}rad/s, limit=1.8rad/s'
    for label,data in (('pose',message.pose.covariance),('twist',message.twist.covariance)):
        covariance = np.array(data).reshape(6,6)
        if not np.allclose(covariance,covariance.T,atol=1e-8):
            return f'{label}_covariance_asymmetry={np.abs(covariance-covariance.T).max():.6g}'
        eigenvalue=float(np.linalg.eigvalsh(covariance).min())
        if eigenvalue < -1e-9:
            return f'{label}_covariance_min_eigenvalue={eigenvalue:.6g}'
    return None


def candidate_valid(message):
    return candidate_issue(message) is None


def plausible_step(first, second, max_gap=.35):
    dt = stamp_seconds(second)-stamp_seconds(first)
    if dt <= 0 or dt > max_gap:
        return True
    p, q = first.pose.pose.position, second.pose.pose.position
    return (math.hypot(q.x-p.x,q.y-p.y) <= .8*dt+.02 and
            abs(math.remainder(yaw(second)-yaw(first),2*math.pi)) <= 1.8*dt+.03)


def step_issue(first, second, max_gap=.35):
    if first is None or plausible_step(first,second,max_gap):
        return None
    dt=stamp_seconds(second)-stamp_seconds(first)
    p,q=first.pose.pose.position,second.pose.pose.position
    distance=math.hypot(q.x-p.x,q.y-p.y)
    turn=abs(math.remainder(yaw(second)-yaw(first),2*math.pi))
    return (f'discontinuous_step dt={dt:.6f}s, distance={distance:.6f}m '
            f'(limit={.8*dt+.02:.6f}), yaw={turn:.6f}rad (limit={1.8*dt+.03:.6f})')


def predict_odometry(message, target_stamp, gyro_history=(), max_dt=.35,
                     acceleration_bound=1.5, yaw_acceleration_bound=2.):
    """短时常车体系速度预测；有覆盖全窗的 gyro 时积分实测角速度。

位置、姿态和协方差一起传播，不能只改消息时间。这里只补偿合法传输延迟，
不续期传感器观测或激光锚点；模型未观测加速度的界限额外计入误差预算。
"""
    begin = stamp_seconds(message)
    dt = target_stamp-begin
    if dt <= 0:
        return deepcopy(message)
    if dt > max_dt+1e-8 or not candidate_valid(message):
        return None
    output = deepcopy(message)
    v = message.twist.twist
    history = list(gyro_history)
    use_gyro = bool(history and history[0][0] <= begin and history[-1][0] >= target_stamp)
    knots = [begin]+([r[0] for r in history if begin < r[0] < target_stamp] if use_gyro else [])+[target_stamp]
    def angular(stamp):
        if use_gyro:
            for a,b in zip(history,history[1:]):
                if a[0] <= stamp <= b[0]:
                    f=(stamp-a[0])/(b[0]-a[0])
                    return (a[1]+f*(b[1]-a[1]),a[2]+f*(b[2]-a[2]))
        return v.angular.z,message.twist.covariance[35]
    angle = yaw(message)
    dx = dy = 0.
    for start,end in zip(knots,knots[1:]):
        step=end-start
        rate=(angular(start)[0]+angular(end)[0])*.5
        turn=rate*step
        a=math.sin(turn)/rate if abs(rate)>1e-9 else step
        b=(1.-math.cos(turn))/rate if abs(rate)>1e-9 else 0.
        bx=a*v.linear.x-b*v.linear.y
        by=b*v.linear.x+a*v.linear.y
        dx+=math.cos(angle)*bx-math.sin(angle)*by
        dy+=math.sin(angle)*bx+math.cos(angle)*by
        angle+=turn
    output.pose.pose.position.x+=dx
    output.pose.pose.position.y+=dy
    output.pose.pose.orientation.z,output.pose.pose.orientation.w=math.sin(angle/2),math.cos(angle/2)
    output.twist.twist.angular.z=angular(target_stamp)[0]
    covariance=np.array(message.pose.covariance).reshape(6,6)
    twist_covariance=np.array(message.twist.covariance).reshape(6,6)
    twist_covariance[5,5]=max(twist_covariance[5,5],angular(target_stamp)[1])
    transition=np.eye(6);transition[0,5]=-dy;transition[1,5]=dx
    influence=np.zeros((6,6));c,s=math.cos(yaw(message)),math.sin(yaw(message))
    influence[:2,:2]=np.array([[c,-s],[s,c]])*dt
    influence[0,5]=-dy*dt*.5;influence[1,5]=dx*dt*.5;influence[5,5]=dt
    covariance=transition@covariance@transition.T+influence@twist_covariance@influence.T
    for axis in (0,1):
        covariance[axis,axis]+=(.5*acceleration_bound*dt*dt)**2
        twist_covariance[axis,axis]+=(acceleration_bound*dt)**2
    covariance[5,5]+=(.5*yaw_acceleration_bound*dt*dt)**2
    output.pose.covariance=covariance.ravel().tolist()
    output.twist.covariance=twist_covariance.ravel().tolist()
    set_stamp(output,target_stamp)
    return output


def consistent_output(filtered, laser, max_position_error, max_yaw_error):
    p, q = filtered.pose.pose.position, laser.pose.pose.position
    v = filtered.twist.twist
    values = [p.x,p.y,p.z,yaw(filtered),v.linear.x,v.linear.y,v.angular.z,
              *filtered.pose.covariance,*filtered.twist.covariance]
    return (all(math.isfinite(x) for x in values)
            and math.hypot(p.x-q.x,p.y-q.y) <= max_position_error
            and abs(math.remainder(yaw(filtered)-yaw(laser),2*math.pi)) <= max_yaw_error
            and math.hypot(v.linear.x,v.linear.y) <= .8 and abs(v.angular.z) <= 1.8)


def interpolate_odometry(history, stamp, max_gap=.35):
    """只在有两端观测的区间内插值，不将正常传输延迟当作定位跳变。"""
    for first, second in zip(history, list(history)[1:]):
        a, b = stamp_seconds(first), stamp_seconds(second)
        if a <= stamp <= b and 0 < b-a <= max_gap:
            f = (stamp-a)/(b-a)
            result = deepcopy(first)
            for axis in ('x', 'y', 'z'):
                setattr(result.pose.pose.position, axis,
                    (1-f)*getattr(first.pose.pose.position,axis)+f*getattr(second.pose.pose.position,axis))
            angle = yaw(first)+f*math.remainder(yaw(second)-yaw(first),2*math.pi)
            result.pose.pose.orientation.z, result.pose.pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
            set_stamp(result,stamp)
            return result
    for item in history:
        if abs(stamp_seconds(item)-stamp) < 1e-6:
            return item
    return None


def aligned_consistency(filtered_history, laser_history, max_position_error, max_yaw_error):
    if not filtered_history or not laser_history:
        return None
    stamp = min(stamp_seconds(filtered_history[-1]), stamp_seconds(laser_history[-1]))
    first = interpolate_odometry(filtered_history, stamp)
    second = interpolate_odometry(laser_history, stamp)
    if first is None or second is None:
        return None
    return consistent_output(first, second, max_position_error, max_yaw_error)


def inertial_covariance_valid(message, max_xy, max_yaw):
    p, v, covariance = message.pose.pose.position, message.twist.twist, message.pose.covariance
    values = [p.x, p.y, p.z, yaw(message), v.linear.x, v.linear.y, v.angular.z,
              *covariance, *message.twist.covariance]
    q = message.pose.pose.orientation
    return (all(math.isfinite(value) for value in values)
            and abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) < .01
            and np.allclose(np.array(covariance).reshape(6,6)[np.ix_([0,1,5],[0,1,5])],
                            np.array(covariance).reshape(6,6)[np.ix_([0,1,5],[0,1,5])].T)
            and np.linalg.eigvalsh(np.array(covariance).reshape(6,6)[np.ix_([0,1,5],[0,1,5])]).min() >= -1e-9
            and all(covariance[i] > 0 for i in (0,7))
            and np.linalg.eigvalsh(np.array(covariance).reshape(6,6)[:2,:2]).max() <= max_xy
            and 0 < covariance[35] <= max_yaw
            and math.hypot(v.linear.x, v.linear.y) <= .8 and abs(v.angular.z) <= 1.8)


class EstimationGuard(Node):
    def __init__(self):
        super().__init__('tracked_estimation_guard')
        def param(name, default):
            return self.declare_parameter(name,default).value
        self.max_age = float(param('max_prediction_s', .2))
        self.max_source_age = float(param('max_measurement_age_s', .35))
        self.recovery_duration = float(param('recovery_duration_s', .3))
        self.max_error = float(param('max_position_error_m', .25))
        self.max_handoff_error = float(param('max_handoff_position_error_m', .5))
        self.max_yaw_error = float(param('max_yaw_error_rad', .15))
        if not 0 < self.max_age <= .5 or not .1 <= self.recovery_duration <= 2:
            raise ValueError('invalid estimation time bounds')
        if (not 0 < self.max_error <= .5 or
                not self.max_error <= self.max_handoff_error <= .75 or
                not 0 < self.max_yaw_error <= .3):
            raise ValueError('invalid estimation consistency bounds')
        if not self.max_age <= self.max_source_age <= 1.:
            raise ValueError('invalid measurement age bound')
        self.inertial_enabled = bool(param('inertial_enabled', False))
        self.max_coast = float(param('max_inertial_duration_s', 2.0))
        self.max_laser_fallback = float(param('max_laser_fallback_s', 2.0))
        self.max_prediction_hold = float(param('max_prediction_hold_s', .45))
        self.max_inertial_xy = float(param('max_inertial_xy_variance', 4.0))
        self.max_inertial_yaw = float(param('max_inertial_yaw_variance', .15))
        self.max_inertial_growth = float(param('max_inertial_xy_growth_variance', .09))
        self.max_inertial_yaw_growth = float(param('max_inertial_yaw_growth_variance', .05))
        if not 0 < self.max_inertial_growth <= .25 or not 0 < self.max_inertial_yaw_growth <= .25:
            raise ValueError('invalid inertial incremental uncertainty bound')
        self.trusted_xy_variance = 0.
        self.trusted_yaw_variance = 0.
        if (not 0 < self.max_coast <= 5. or not 0 < self.max_laser_fallback <= 5.
                or not .1 <= self.max_prediction_hold <= .5 or not all(math.isfinite(v) and v > 0
                for v in (self.max_inertial_xy,self.max_inertial_yaw,self.max_inertial_yaw_growth))):
            raise ValueError('invalid bounded inertial fallback limits')
        self.health_reason = 'waiting_sensors'
        self.observation_reason = 'waiting_sensors'
        self.pause_reason = None
        self.filtered_epoch_stamp = None
        self.filtered_reject_reason = 'waiting_filtered_output'
        self.odom_frame = param('odom_frame','odom')
        self.base_frame = param('base_frame','base_footprint')
        source = param('input_topic','/chassis/odometry/filtered_unchecked')
        output = param('output_topic','/odometry/filtered')
        if source == output:
            raise ValueError('unchecked and public odometry must differ')
        qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # 内部就绪判断需要新的持续心跳，不能用持久历史补发刷新收到时间。
        heartbeat_qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE,
                                   reliability=ReliabilityPolicy.RELIABLE)
        self.valid_pub = self.create_publisher(Bool,param('valid_topic','/chassis/estimation/valid'),qos)
        self.state_pub = self.create_publisher(String,param('state_topic','/chassis/estimation/state'),qos)
        self.publisher = self.create_publisher(Odometry,output,10)
        self.tf = TransformBroadcaster(self)
        self.laser = self.gyro = self.inertial = None
        self.inertial_at = None
        self.last_trusted_laser_stamp = None
        self.ever_valid = False
        self.laser_at = self.gyro_at = None
        self.flags = {}
        self.subs = [self.create_subscription(Odometry,param('laser_topic','/chassis/odom/laser'),self.on_laser,10),
                     self.create_subscription(Imu,param('gyro_topic','/chassis/imu/gyro'),self.on_gyro,qos_profile_sensor_data),
                     self.create_subscription(Odometry,source,self.on_filtered,10)]
        for key, topic in [('imu',param('imu_ready_topic','/chassis/imu/data_ready')),
                           ('laser',param('laser_ready_topic','/chassis/odom/laser_ready'))]:
            self.subs.append(self.create_subscription(Bool,topic,
                lambda m,k=key:self.on_flag(k,m),heartbeat_qos))
        if self.inertial_enabled:
            inertial_topic = param('inertial_topic', '/chassis/odom/inertial')
            if inertial_topic in (source, output):
                raise ValueError('inertial fallback must use an independent topic')
            self.subs.append(self.create_subscription(Odometry, inertial_topic,
                                                       self.on_inertial, 10))
            self.subs.append(self.create_subscription(Bool,
                param('inertial_valid_topic', '/chassis/inertial/valid'),
                lambda m: self.on_flag('inertial', m), heartbeat_qos))
        self.toggle = self.create_client(ToggleFilterProcessing,param('toggle_service','/chassis/ekf/toggle'))
        self.set_pose = self.create_client(SetPose,param('set_pose_service','/chassis/ekf/set_pose'))
        self.mode = 'PAUSING'
        self.future = None
        self.future_operation = None
        self.request_started = None
        self.recovery_since = None
        self.enabled_at = None
        self.last_forwarded = None
        self.valid = False
        self.last_state = None
        self.inertial_output = False
        self.laser_output = False
        self.laser_since = None
        self.laser_fallback_exhausted = False
        self.prediction_anchor = None
        self.fallback_reason = None
        self.filtered_history, self.laser_history = deque(maxlen=120), deque(maxlen=50)
        self.gyro_history = deque(maxlen=200)
        self.last_source_message = {}
        self.prediction_acceleration_bound = float(param('prediction_acceleration_bound_m_s2',1.5))
        self.prediction_yaw_acceleration_bound = float(param('prediction_yaw_acceleration_bound_rad_s2',2.))
        if not (0 < self.prediction_acceleration_bound <= 3. and
                0 < self.prediction_yaw_acceleration_bound <= 5.):
            raise ValueError('invalid delay prediction acceleration bounds')
        self.inconsistent_since = None
        self.handoff_started = None
        self.last_output = None
        self.output_source = None
        self.output_offset = [0., 0., 0.]
        self.correction_speed = float(param('handoff_correction_speed_m_s', .05))
        self.correction_yaw_rate = float(param('handoff_correction_yaw_rate_rad_s', .10))
        if not 0 < self.correction_speed <= .1 or not 0 < self.correction_yaw_rate <= .2:
            raise ValueError('invalid handoff correction limits')
        self.timer = self.create_timer(.05,self.tick)
        self.tick()

    def on_flag(self,key,message):
        self.flags[key] = (bool(message.data),time.monotonic())

    def on_laser(self,message):
        if message.header.frame_id == self.odom_frame and message.child_frame_id == self.base_frame:
            self.laser,self.laser_at = message,time.monotonic()
            if not self.laser_history or stamp_seconds(message) > stamp_seconds(self.laser_history[-1]):
                self.laser_history.append(message)
            if self.laser_output and self.laser_usable():
                self.forward_laser()

    def on_gyro(self,message):
        if message.header.frame_id == self.base_frame and math.isfinite(message.angular_velocity.z):
            self.gyro,self.gyro_at = message,time.monotonic()
            stamp=stamp_seconds(message)
            if not self.gyro_history or stamp > self.gyro_history[-1][0]:
                variance=float(message.angular_velocity_covariance[8])
                if math.isfinite(variance) and variance >= 0 and abs(message.angular_velocity.z) <= 1.8:
                    self.gyro_history.append((stamp,message.angular_velocity.z,max(.001,variance)))

    def freshness_issue(self,message,received):
        if message is None or received is None:
            return 'missing_message'
        age = self.get_clock().now().nanoseconds*1e-9-stamp_seconds(message)
        receipt_age=time.monotonic()-received
        if not -.05 <= age <= self.max_source_age or receipt_age > self.max_source_age:
            return (f'header_age={age:.6f}s (allowed=-0.05..{self.max_source_age:.3f}), '
                    f'receipt_age={receipt_age:.6f}s (limit={self.max_source_age:.3f})')
        return None

    def fresh(self,message,received):
        return self.freshness_issue(message,received) is None

    def pause(self,reason):
        # 第一触发原因保留至成功恢复；healthy() 只能更新观测状态，不能抹掉故障。
        if self.pause_reason is None:
            self.pause_reason=reason
        self.health_reason=self.pause_reason
        self.valid=self.inertial_output=self.laser_output=False
        self.laser_since=None
        self.mode='PAUSING'

    def prediction_usable(self):
        """只允许从进入故障时的可信公开位姿预测，不能用自身输出续期。"""
        if not self.ever_valid or self.last_output is None or self.last_forwarded is None:
            return False
        flag = self.flags.get('imu')
        if (flag is None or not flag[0] or time.monotonic()-flag[1] >= .5
                or not self.fresh(self.gyro, self.gyro_at)):
            return False
        if self.prediction_anchor is None and time.monotonic()-self.last_forwarded[1] > self.max_age:
            return False
        anchor = self.prediction_anchor or self.last_output
        target = min(self.get_clock().now().nanoseconds*1e-9, stamp_seconds(self.gyro))
        return 0 <= target-stamp_seconds(anchor) <= self.max_prediction_hold

    def use_prediction(self, reason):
        if not self.prediction_usable():
            return False
        if self.prediction_anchor is None:
            self.prediction_anchor = deepcopy(self.last_output)
        self.mode = 'PREDICTED'
        self.inertial_output = self.laser_output = False
        self.fallback_reason = reason
        self.filtered_reject_reason = reason
        candidate = predict_odometry(
            self.prediction_anchor,
            min(self.get_clock().now().nanoseconds*1e-9, stamp_seconds(self.gyro)),
            self.gyro_history, self.max_prediction_hold,
            self.prediction_acceleration_bound, self.prediction_yaw_acceleration_bound)
        if candidate is None:
            return False
        if stamp_seconds(candidate) > stamp_seconds(self.last_output):
            if not self.forward(candidate, 'predicted'):
                return False
        self.valid = (self.last_forwarded is not None and
                      time.monotonic()-self.last_forwarded[1] <= self.max_age)
        return self.valid

    def laser_usable(self):
        """激光质量独立确认后，允许 EKF 异常时以单源里程计短时续接。"""
        flag = self.flags.get('laser')
        return (self.ever_valid and not self.laser_fallback_exhausted
                and flag is not None and flag[0]
                and time.monotonic()-flag[1] < .5
                and self.fresh(self.laser, self.laser_at)
                and candidate_valid(self.laser))

    def predicted_laser(self):
        """用新鲜的质量合格激光观测短时传播到当前 TF 时刻。"""
        if not self.laser_usable():
            return None
        target = self.get_clock().now().nanoseconds*1e-9
        laser_stamp = stamp_seconds(self.laser)
        if self.fresh(self.gyro, self.gyro_at) and stamp_seconds(self.gyro) >= laser_stamp:
            target = min(target, stamp_seconds(self.gyro))
        return predict_odometry(self.laser, target, self.gyro_history, self.max_source_age,
                                self.prediction_acceleration_bound,
                                self.prediction_yaw_acceleration_bound)

    def forward_laser(self):
        candidate = self.predicted_laser()
        if candidate is None:
            return False
        issue = candidate_issue(candidate) or step_issue(
            self.last_source_message.get('laser'), candidate, self.max_source_age)
        if issue is not None:
            self.filtered_reject_reason = 'laser_output_candidate_invalid: '+issue
            return False
        return self.forward(candidate, 'laser')

    def use_laser(self, reason):
        if not self.laser_usable():
            self.pause(reason)
            return False
        self.prediction_anchor = None
        if self.laser_since is None:
            self.laser_since = time.monotonic()
        self.mode = 'LASER'
        self.laser_output = True
        self.valid = (self.last_forwarded is not None and
                      time.monotonic()-self.last_forwarded[1] <= self.max_age)
        self.inertial_output = False
        self.fallback_reason = reason
        self.filtered_reject_reason = reason
        self.filtered_history.clear()
        self.recovery_since = None
        self.handoff_started = None
        if self.forward_laser():
            self.valid = True
        if self.mode == 'PAUSING':
            return False
        return True

    def use_inertial(self, reason):
        if not self.inertial_usable():
            return False
        self.prediction_anchor = None
        self.mode = 'INERTIAL'
        self.inertial_output = True
        self.laser_output = False
        self.laser_since = None
        self.fallback_reason = reason
        self.filtered_reject_reason = reason
        self.filtered_history.clear()
        self.recovery_since = None
        self.handoff_started = None
        self.valid = (self.last_forwarded is not None and
                      time.monotonic()-self.last_forwarded[1] <= self.max_age)
        if self.forward(self.inertial, 'inertial'):
            self.valid = True
        return self.mode != 'PAUSING'

    def isolate_filtered_candidate(self, reason):
        """隔离异常 EKF 帧，优先选用仍独立有效的观测续接。"""
        if self.laser_usable():
            return self.use_laser(reason)
        if self.use_inertial(reason):
            return True
        if self.use_prediction(reason):
            return True
        self.pause(reason + '; fallback_unavailable')
        return False

    def healthy(self):
        now = time.monotonic()
        reason=None
        for key in ['imu','laser']:
            if key not in self.flags or not self.flags[key][0] or now-self.flags[key][1]>=.5:
                flag=self.flags.get(key)
                detail='missing_heartbeat' if flag is None else f'value={flag[0]}, heartbeat_age={now-flag[1]:.6f}s'
                reason=key+'_not_ready: '+detail
                break
        if reason is None:
            for key,message,received in [('laser',self.laser,self.laser_at),('gyro',self.gyro,self.gyro_at)]:
                issue=self.freshness_issue(message,received)
                if issue is not None:
                    reason=key+'_stale: '+issue
                    break
        if reason is None:
            issue=candidate_issue(self.laser)
            if issue is not None:
                reason='laser_candidate_invalid: '+issue
        self.observation_reason=reason or 'observations_ready'
        self.health_reason=self.pause_reason or self.observation_reason
        return reason is None

    def on_inertial(self, message):
        if message.header.frame_id == self.odom_frame and message.child_frame_id == self.base_frame:
            self.inertial, self.inertial_at = message, time.monotonic()
            if self.inertial_output and self.handoff_started is None and self.inertial_usable():
                self.forward(message, 'inertial')

    def inertial_usable(self):
        # valid 心跳不能独自授权：仍检查 IMU、方差及最后可信激光时间。
        if not self.inertial_enabled or not self.ever_valid:
            return False
        now = time.monotonic()
        for key in ('imu', 'inertial'):
            if key not in self.flags or not self.flags[key][0] or now-self.flags[key][1] >= .5:
                return False
        if not self.fresh(self.gyro, self.gyro_at) or not self.fresh(self.inertial, self.inertial_at):
            return False
        if self.last_trusted_laser_stamp is None:
            return False
        reference_stamp=self.last_trusted_laser_stamp
        baseline_xy,baseline_yaw=self.trusted_xy_variance,self.trusted_yaw_variance
        # 动态协方差也必须对齐时间：新激光先到且方差变小时，不能拿它
        # 较小的未来基线拒绝仍继承上一帧误差的 INS。
        cutoff=min(stamp_seconds(self.inertial),reference_stamp)
        for reference in reversed(self.laser_history):
            if stamp_seconds(reference) <= cutoff+1e-8 and candidate_valid(reference):
                reference_stamp=stamp_seconds(reference)
                covariance=np.array(reference.pose.covariance).reshape(6,6)
                baseline_xy=float(np.linalg.eigvalsh(covariance[:2,:2]).max())
                baseline_yaw=float(covariance[5,5])
                break
        dt = stamp_seconds(self.inertial)-reference_stamp
        # 12 Hz 激光可能比最新 30 Hz INS 领先一帧；不是负的续推时长。
        # INS 自身的 valid 仍独立检查完整锚点期限，不能借此续期。
        return (-self.max_age <= dt <= self.max_coast and inertial_covariance_valid(
            self.inertial,
            min(self.max_inertial_xy,baseline_xy+self.max_inertial_growth)
                if baseline_xy <= self.max_inertial_xy else baseline_xy+self.max_inertial_growth,
            min(self.max_inertial_yaw,baseline_yaw+self.max_inertial_yaw_growth)
                if baseline_yaw <= self.max_inertial_yaw else baseline_yaw+self.max_inertial_yaw_growth))

    def publish_state(self):
        state = ('LASER' if self.laser_output and self.valid else
                 'INERTIAL' if self.inertial_output and self.valid else self.mode)
        self.valid_pub.publish(Bool(data=self.valid))
        self.state_pub.publish(String(data=state))
        if state != self.last_state:
            detail=self.fallback_reason if state in ('LASER', 'PREDICTED') else self.health_reason
            if self.pause_reason and self.observation_reason != self.pause_reason:
                detail+='; current_observations='+self.observation_reason
            self.get_logger().info('Estimation state: '+state+' ('+detail+')')
            self.last_state = state

    def request_toggle(self, on):
        if self.toggle.service_is_ready():
            self.future = self.toggle.call_async(ToggleFilterProcessing.Request(on=on))
            self.future_operation = 'enable' if on else 'pause'
            self.request_started = time.monotonic()

    def request_reset(self):
        if self.set_pose.service_is_ready():
            request = SetPose.Request()
            request.pose.header = self.laser.header
            request.pose.pose = self.laser.pose
            self.future = self.set_pose.call_async(request)
            self.future_operation = 'reset'
            self.request_started = time.monotonic()
            self.mode = 'RESETTING'

    def begin_filtered_epoch(self):
        # SetPose 已明确造成不连续的新滤波时期。以确认时刻隔离排队旧帧，
        # 不能拿重置后的真实位姿与重置前的插值/步长历史继续做连续性判断。
        stamp=self.get_clock().now().nanoseconds*1e-9
        self.filtered_epoch_stamp=max(stamp,self.filtered_epoch_stamp or stamp)
        self.filtered_history.clear()
        self.last_source_message.clear()
        self.last_output=self.output_source=None
        self.output_offset=[0.,0.,0.]
        self.inconsistent_since=self.handoff_started=None
        self.valid=self.inertial_output=self.laser_output=False
        self.laser_since=None
        self.laser_fallback_exhausted=False
        self.prediction_anchor=None
        self.fallback_reason=None
        self.pause_reason=None
        self.health_reason='ekf_reset_confirmed_waiting_new_epoch'
        self.filtered_reject_reason='waiting_post_reset_output'
        # last_forwarded 保留：即使显式重定位，公共 TF 时间也不能倒退。

    def finish_request(self, now):
        if self.future is None:
            return
        if not self.future.done():
            if now-self.request_started <= 2.:
                return
            self.future.cancel()
            self.get_logger().error('EKF service timeout; confirming pause before recovery')
            self.pause('ekf_service_timeout: '+str(self.future_operation))
        else:
            try:
                result = self.future.result()
                if result is None:
                    raise RuntimeError('EKF service returned no response')
                # Humble ToggleFilterProcessing.status=false 表示已是请求状态，
                # 并非失败。短时惯导期间 EKF 保持开启，恢复 enable 是幂等确认。
                if self.future_operation == 'reset':
                    self.begin_filtered_epoch()
                self.mode = {'pause': 'PAUSED', 'reset': 'ENABLING',
                             'enable': 'WARMING'}[self.future_operation]
                if self.mode == 'WARMING':
                    self.enabled_at = now
            except Exception as error:
                self.get_logger().error('EKF service failure: '+str(error))
                self.pause('ekf_service_failure: '+str(error))
        self.future = None
        self.future_operation = None

    def tick(self):
        now = time.monotonic()
        self.finish_request(now)
        healthy = self.healthy()
        if healthy:
            self.last_trusted_laser_stamp = stamp_seconds(self.laser)
            covariance = np.array(self.laser.pose.covariance).reshape(6,6)
            self.trusted_xy_variance = float(np.linalg.eigvalsh(covariance[:2,:2]).max())
            self.trusted_yaw_variance = float(covariance[5,5])
        can_coast = self.inertial_usable()
        can_laser = self.laser_usable()
        if not healthy:
            self.recovery_since = None
            self.handoff_started = None
            self.inconsistent_since = None
            if can_laser and self.mode not in ('PAUSING', 'PAUSED'):
                self.prediction_anchor = None
                if self.laser_since is None:
                    self.laser_since = now
                self.mode = 'LASER'
                self.laser_output = True
                self.valid = (self.last_forwarded is not None and
                              now-self.last_forwarded[1] <= self.max_age)
                self.inertial_output = False
                self.fallback_reason = self.observation_reason
            elif can_coast and self.mode not in ('PAUSING', 'PAUSED'):
                self.prediction_anchor = None
                # 保留 EKF 运行；公共输出选独立 INS，不把同源积分反灌融合。
                self.inertial_output = True
                self.laser_output = False
                self.laser_since = None
                self.mode = 'INERTIAL' if self.future is None else self.mode
                self.valid = True
            elif self.mode not in ('PAUSING', 'PAUSED') and self.use_prediction(self.observation_reason):
                pass
            else:
                self.valid = self.inertial_output = self.laser_output = False
                reason = ('prediction_window_or_gyro_expired' if self.mode == 'PREDICTED'
                          else self.observation_reason+'; inertial_unavailable_or_expired')
                if self.future is None:
                    if self.mode != 'PAUSED':
                        self.pause(reason)
                if self.pause_reason is None:
                    self.pause_reason=reason
                self.health_reason=self.pause_reason
        elif self.mode in ('PAUSED', 'INERTIAL', 'LASER', 'PREDICTED'):
            if self.recovery_since is None:
                self.recovery_since = now
            if (self.mode == 'PAUSED' and now-self.recovery_since >= self.recovery_duration
                    and self.future is None):
                self.request_reset()
            # INERTIAL期间EKF一直运行，由同时间创新检验直接接管，不清零速度。
        elif self.mode == 'ENABLING' and self.future is None:
            self.request_toggle(True)
        if (self.mode == 'WARMING' and self.enabled_at is not None
                and now-self.enabled_at > 3.0):
            self.pause('ekf_output_recovery_timeout: '+self.filtered_reject_reason)
        if self.mode == 'PAUSING' and self.future is None:
            self.valid = self.inertial_output = False
            self.request_toggle(False)
        if self.inertial_output:
            if can_coast:
                self.valid = True
                if self.handoff_started is None:
                    self.forward(self.inertial, 'inertial')
            else:
                self.valid = self.inertial_output = False
        if self.laser_output:
            if self.laser_since is not None and now-self.laser_since > self.max_laser_fallback:
                reason='laser_only_duration_exceeded: '+str(self.fallback_reason)
                self.laser_fallback_exhausted = True
                if not self.use_inertial(reason):
                    self.pause(reason)
            elif can_laser:
                if self.forward_laser():
                    self.valid = True
            else:
                self.valid = self.laser_output = False
                self.laser_since = None
        if self.mode == 'PREDICTED':
            if self.prediction_usable():
                self.use_prediction(self.fallback_reason or self.observation_reason)
            elif not self.use_inertial('prediction_window_or_gyro_expired'):
                self.pause('prediction_window_or_gyro_expired')
        if self.valid and (self.last_forwarded is None or now-self.last_forwarded[1] > self.max_age):
            self.valid = False
            if self.future is None:
                age=math.inf if self.last_forwarded is None else now-self.last_forwarded[1]
                reason=(f'output_stale: age={age:.3f}s, limit={self.max_age:.3f}s; '
                        +self.filtered_reject_reason)
                if self.mode in ('VALID', 'WARMING'):
                    self.isolate_filtered_candidate(reason)
                elif self.mode == 'LASER':
                    self.use_inertial(reason)
                if not self.valid:
                    self.pause(reason)
        self.publish_state()

    def on_filtered(self, message):
        # 暂停/服务处理中来的 EKF 输出可能属于旧时期，不能改变恢复状态机，
        # 也不能先写进连续性历史。只有启用后的 WARMING 才开始核验新候选。
        if self.mode not in ('WARMING', 'VALID', 'INERTIAL', 'LASER', 'PREDICTED'):
            return
        now = time.monotonic()
        if self.filtered_epoch_stamp is not None and stamp_seconds(message) <= self.filtered_epoch_stamp:
            self.filtered_reject_reason=(f'pre_reset_ekf_frame: stamp={stamp_seconds(message):.6f}, '
                                         f'epoch={self.filtered_epoch_stamp:.6f}')
            return
        if message.header.frame_id != self.odom_frame or message.child_frame_id != self.base_frame:
            self.isolate_filtered_candidate(
                'ekf_frame_mismatch: '+message.header.frame_id+'->'+message.child_frame_id)
            return
        issue=self.freshness_issue(message,now)
        if issue is not None:
            self.filtered_reject_reason='ekf_stale: '+issue
            return
        if self.filtered_history and stamp_seconds(message) <= stamp_seconds(self.filtered_history[-1]):
            self.filtered_reject_reason='ekf_stamp_not_increasing'
            return
        issue=candidate_issue(message) or step_issue(
            self.filtered_history[-1] if self.filtered_history else None,message,self.max_source_age)
        if issue is not None:
            self.isolate_filtered_candidate('ekf_candidate_invalid: '+issue)
            return
        if not self.filtered_history or stamp_seconds(message) > stamp_seconds(self.filtered_history[-1]):
            self.filtered_history.append(message)
        self.filtered_reject_reason=''
        if not self.healthy():
            self.filtered_reject_reason=self.observation_reason
            return
        consistency = aligned_consistency(self.filtered_history, self.laser_history,
                                          self.max_error, self.max_yaw_error)
        if consistency is None:
            self.filtered_reject_reason='awaiting_time_aligned_ekf_laser'
            return
        if not consistency:
            self.filtered_reject_reason='ekf_laser_aligned_innovation_exceeds_bound'
            if self.inconsistent_since is None:
                self.inconsistent_since = now
            output_age = (math.inf if self.last_forwarded is None else
                          now-self.last_forwarded[1])
            if (now-self.inconsistent_since >= self.recovery_duration or
                    output_age >= .5*self.max_age):
                self.get_logger().warning('EKF output inconsistent at aligned timestamps; isolating candidate',
                                          throttle_duration_sec=5.)
                self.isolate_filtered_candidate(self.filtered_reject_reason)
            return
        self.inconsistent_since = None
        if self.mode == 'WARMING' and now-self.enabled_at < self.recovery_duration:
            return
        if self.mode in ('INERTIAL', 'LASER', 'PREDICTED'):
            if self.recovery_since is None:
                self.recovery_since = now
            duration = min(.1, self.recovery_duration) if self.mode == 'PREDICTED' else self.recovery_duration
            if now-self.recovery_since < duration:
                return
        # 持续 250 ms 合法迟到时，不能先停 INS 等 EKF 的旧时间戳追上。
        # 用实测 gyro / 有界常速度模型补齐到当前采样时刻，并传播误差。
        target_stamp=min(self.get_clock().now().nanoseconds*1e-9,stamp_seconds(self.gyro))
        candidate=predict_odometry(message,target_stamp,self.gyro_history,self.max_source_age,
                                   self.prediction_acceleration_bound,self.prediction_yaw_acceleration_bound)
        if candidate is None:
            self.filtered_reject_reason='ekf_delay_prediction_unavailable'
            return
        if self.inertial_output and self.handoff_started is None:
            # 已有当前采样时刻的完整预测候选；只让出下一帧发布权，
            # 不再等待延迟的原始 EKF 时间戳追赶数百毫秒。
            self.handoff_started=now
        adopted = False
        if (self.mode == 'PREDICTED' and self.last_output is not None and
                abs(stamp_seconds(candidate)-stamp_seconds(self.last_output)) < 1e-8):
            old = self.last_output.pose.pose.position
            new = candidate.pose.pose.position
            offset = [old.x-new.x, old.y-new.y,
                      math.remainder(yaw(self.last_output)-yaw(candidate), 2*math.pi)]
            if math.hypot(*offset[:2]) > self.max_handoff_error or abs(offset[2]) > self.max_yaw_error:
                self.pause(f'handoff_innovation_exceeds_bound: xy={math.hypot(*offset[:2]):.6f}m '
                           f'(limit={self.max_handoff_error}), yaw={offset[2]:.6f}rad '
                           f'(limit={self.max_yaw_error})')
                return
            # 这一时间戳已发布预测位姿：只建立连续交接偏移，下一帧再发布。
            self.output_offset = offset
            self.output_source = 'filtered'
            self.last_source_message['filtered'] = deepcopy(candidate)
            adopted = True
        if adopted or self.forward(candidate, 'filtered'):
            self.mode = 'VALID'
            self.pause_reason=None
            self.health_reason=self.observation_reason
            self.valid = self.ever_valid = True
            self.inertial_output = False
            self.laser_output = False
            self.laser_since = None
            self.laser_fallback_exhausted = False
            self.prediction_anchor = None
            self.fallback_reason = None
            self.handoff_started = None
            self.last_trusted_laser_stamp = stamp_seconds(self.laser)
            self.recovery_since = None
            covariance=np.array(self.laser.pose.covariance).reshape(6,6)
            self.trusted_xy_variance=float(np.linalg.eigvalsh(covariance[:2,:2]).max())
            self.trusted_yaw_variance=float(covariance[5,5])

    def forward(self, message, source='filtered'):
        stamp = stamp_seconds(message)
        if self.last_forwarded is not None and stamp <= self.last_forwarded[0]:
            return False
        previous=self.last_source_message.get(source)
        issue=candidate_issue(message) or step_issue(previous,message,self.max_source_age)
        if issue is not None:
            self.pause(source+'_output_candidate_invalid: '+issue)
            return False
        output = deepcopy(message)
        angle = yaw(message)
        correction = [0., 0., 0.]
        if self.last_output is not None:
            dt = stamp-stamp_seconds(self.last_output)
            if self.output_source != source:
                old = self.last_output
                old_yaw = yaw(old)
                old_v = old.twist.twist
                if dt <= self.max_source_age:
                    px = old.pose.pose.position.x+dt*(math.cos(old_yaw)*old_v.linear.x-math.sin(old_yaw)*old_v.linear.y)
                    py = old.pose.pose.position.y+dt*(math.sin(old_yaw)*old_v.linear.x+math.cos(old_yaw)*old_v.linear.y)
                    offset = [px-message.pose.pose.position.x, py-message.pose.pose.position.y,
                              math.remainder(old_yaw+dt*old_v.angular.z-angle,2*math.pi)]
                    if math.hypot(*offset[:2]) > self.max_handoff_error or abs(offset[2]) > self.max_yaw_error:
                        self.pause(f'handoff_innovation_exceeds_bound: xy={math.hypot(*offset[:2]):.6f}m '
                                   f'(limit={self.max_handoff_error}), yaw={offset[2]:.6f}rad '
                                   f'(limit={self.max_yaw_error})')
                        return False
                    self.output_offset = offset
                else:
                    # 显式无效区间后，允许经过旧参考核验的重定位。
                    self.output_offset = [0., 0., 0.]
            elif source == 'filtered' and dt <= self.max_source_age:
                length = math.hypot(*self.output_offset[:2])
                fraction = min(1., self.correction_speed*dt/max(length,1e-12))
                correction[:2] = [-x*fraction for x in self.output_offset[:2]]
                correction[2] = -max(-self.correction_yaw_rate*dt,
                                     min(self.correction_yaw_rate*dt,self.output_offset[2]))
                self.output_offset = [a+b for a,b in zip(self.output_offset,correction)]
            if dt > 0:
                correction = [x/dt for x in correction]
        else:
            self.output_offset = [0., 0., 0.]
        output.pose.pose.position.x += self.output_offset[0]
        output.pose.pose.position.y += self.output_offset[1]
        out_yaw = angle+self.output_offset[2]
        output.pose.pose.orientation.z, output.pose.pose.orientation.w = math.sin(out_yaw/2), math.cos(out_yaw/2)
        vx, vy = message.twist.twist.linear.x, message.twist.twist.linear.y
        wx = math.cos(angle)*vx-math.sin(angle)*vy+correction[0]
        wy = math.sin(angle)*vx+math.cos(angle)*vy+correction[1]
        output.twist.twist.linear.x = math.cos(out_yaw)*wx+math.sin(out_yaw)*wy
        output.twist.twist.linear.y = -math.sin(out_yaw)*wx+math.cos(out_yaw)*wy
        output.twist.twist.angular.z += correction[2]
        rotation=np.eye(6)
        c,s=math.cos(self.output_offset[2]),math.sin(self.output_offset[2])
        rotation[:2,:2]=[[c,s],[-s,c]]
        twist_covariance=rotation@np.array(message.twist.covariance).reshape(6,6)@rotation.T
        output.twist.covariance=twist_covariance.ravel().tolist()
        for i, row in enumerate((0,1,5)):
            for j, col in enumerate((0,1,5)):
                output.pose.covariance[row*6+col] += self.output_offset[i]*self.output_offset[j]
        self.last_forwarded = (stamp, time.monotonic())
        self.last_source_message[source]=deepcopy(message)
        self.last_output, self.output_source = output, source
        self.publisher.publish(output)
        transform = TransformStamped()
        transform.header = output.header
        transform.child_frame_id = output.child_frame_id
        transform.transform.translation.x = output.pose.pose.position.x
        transform.transform.translation.y = output.pose.pose.position.y
        transform.transform.translation.z = output.pose.pose.position.z
        transform.transform.rotation = output.pose.pose.orientation
        self.tf.sendTransform(transform)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = EstimationGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
