#!/usr/bin/env python3
"""隔离域合成传感器回归：启动、惯导接管、恢复连续性与地图恢复。

source /opt/ros/humble/setup.bash
python3 tools/test_tracked_recovery.py --output-dir /tmp/kiwi_recovery
python3 tools/test_tracked_recovery.py --domain-id 95 --laser-xy-variance 12.5166
python3 tools/test_tracked_recovery.py --domain-id 96 --turn-rate .10 \
  --laser-delay-s .08 --gyro-delay-s .01 --acceleration-delay-s .025

运行真实 EKF、激光适配器、独立惯导、估计保护、建图门控和 SLAM Toolbox。
只生成合成扫描和 IMU；不连接硬件、不发送底盘运动指令。阈值属于合成软件
回归要求，不代表真实走廊精度。完整真值与输出按各自 ROS 时间戳记录。
"""
import argparse
from bisect import bisect_left, bisect_right
from collections import defaultdict
import csv
import heapq
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_yaw(quaternion):
    return math.atan2(2*(quaternion.w*quaternion.z+quaternion.x*quaternion.y),
                      1-2*(quaternion.y**2+quaternion.z**2))


def distribution(values):
    values = sorted(values)
    if not values:
        return {'count': 0}
    def quantile(p):
        index = (len(values)-1)*p
        lower = math.floor(index)
        upper = math.ceil(index)
        return values[lower]+(values[upper]-values[lower])*(index-lower)
    return {'count': len(values), 'min': values[0], 'p50': quantile(.5),
            'p95': quantile(.95), 'max': values[-1]}


def truth_errors(samples, truth):
    """比较同一 ROS stamp，不拟合坐标、比例或时间偏移。"""
    times = [row['stamp'] for row in truth]
    errors = []
    for row in samples:
        right = bisect_left(times, row['stamp'])
        if right == 0 or right == len(times):
            continue
        a, b = truth[right-1], truth[right]
        dt = b['stamp']-a['stamp']
        if not 0 < dt <= .15:
            continue
        ratio = (row['stamp']-a['stamp'])/dt
        reference = {key: a[key]+(b[key]-a[key])*ratio
                     for key in ('x', 'y', 'vx', 'vy', 'wz')}
        reference['yaw'] = a['yaw']+wrap_angle(b['yaw']-a['yaw'])*ratio
        errors.append({'t': row['t'], 'stamp': row['stamp'],
                       'position_m': math.hypot(row['x']-reference['x'], row['y']-reference['y']),
                       'yaw_rad': abs(wrap_angle(row['yaw']-reference['yaw'])),
                       'vx_m_s': abs(row['vx']-reference['vx']),
                       'vy_m_s': abs(row['vy']-reference['vy']),
                       'wz_rad_s': abs(row['wz']-reference['wz'])})
    metrics = {key: distribution([row[key] for row in errors])
               for key in ('position_m', 'yaw_rad', 'vx_m_s', 'vy_m_s', 'wz_rad_s')}
    metrics['position_rmse_m'] = (sum(row['position_m']**2 for row in errors)/len(errors))**.5 if errors else None
    metrics['compared_samples'] = len(errors)
    metrics['unmatched_samples'] = len(samples)-len(errors)
    return metrics


def continuity_metrics(samples, validity, max_position_base_m=.015,
                       max_position_rate_m_s=.35, max_yaw_base_rad=.025,
                       max_yaw_rate_rad_s=.30):
    """连续有效区间内，比较位姿增量与体坐标 Twist 的梯形积分。

    合法的失效后重置不纳入连续性结论；持续有效时的跳变必须暴露。
    不是单靠最终坐标或发布频率判定成功。
    """
    flag_times = [row['t'] for row in validity]
    comparisons, violations = [], []
    for a, b in zip(samples, samples[1:]):
        dt = b['stamp']-a['stamp']
        if not 0 < dt <= .15:
            continue
        left = bisect_right(flag_times, a['t'])-1
        right = bisect_right(flag_times, b['t'])
        if left < 0 or not validity[left]['value']:
            continue
        if any(not flag['value'] for flag in validity[left+1:right]):
            continue
        def world_velocity(row):
            c, s = math.cos(row['yaw']), math.sin(row['yaw'])
            return c*row['vx']-s*row['vy'], s*row['vx']+c*row['vy']
        va, vb = world_velocity(a), world_velocity(b)
        dx = b['x']-a['x']-(va[0]+vb[0])*.5*dt
        dy = b['y']-a['y']-(va[1]+vb[1])*.5*dt
        position = math.hypot(dx, dy)
        yaw = abs(wrap_angle(b['yaw']-a['yaw']-(a['wz']+b['wz'])*.5*dt))
        item = {'t': b['t'], 'stamp': b['stamp'], 'dt_s': dt,
                'position_residual_m': position, 'velocity_residual_m_s': position/dt,
                'yaw_residual_rad': yaw, 'yaw_rate_residual_rad_s': yaw/dt,
                'position_limit_m': max_position_base_m+max_position_rate_m_s*dt,
                'yaw_limit_rad': max_yaw_base_rad+max_yaw_rate_rad_s*dt}
        comparisons.append(item)
        if position > item['position_limit_m'] or yaw > item['yaw_limit_rad']:
            violations.append(item)
    return {'compared_pairs': len(comparisons), 'violation_count': len(violations),
            'position_residual_m': distribution([r['position_residual_m'] for r in comparisons]),
            'velocity_residual_m_s': distribution([r['velocity_residual_m_s'] for r in comparisons]),
            'yaw_residual_rad': distribution([r['yaw_residual_rad'] for r in comparisons]),
            'largest_events': sorted(comparisons, key=lambda r: r['velocity_residual_m_s'], reverse=True)[:10],
            'violations': violations[:30]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=94)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--startup-timeout', type=float, default=25.)
    parser.add_argument('--laser-xy-variance', type=float, default=None,
                        help='覆盖适配器平面方差下限；12.5166 复现实车继承的大协方差')
    parser.add_argument('--turn-rate', type=float, default=0.,
                        help='运动阶段目标角速度，rad/s；0.10 用于曲线运动回归')
    parser.add_argument('--laser-delay-s', type=float, default=0.,
                        help='仅延迟原始激光里程计到达，保留采集时间戳')
    parser.add_argument('--gyro-delay-s', type=float, default=0.)
    parser.add_argument('--acceleration-delay-s', type=float, default=0.)
    args = parser.parse_args()
    if args.domain_id in (0, 42, 43) or not 1 <= args.domain_id <= 101:
        parser.error('请选择 1–101 的隔离域，禁止 0、42、43 实车常用域')
    if not math.isfinite(args.turn_rate) or abs(args.turn_rate) > .2:
        parser.error('合成转向速度绝对值必须 <= 0.2 rad/s')
    if args.laser_xy_variance is not None and not 0 < args.laser_xy_variance <= 100.:
        parser.error('laser-xy-variance 必须为有限的 (0, 100] m²')
    for name in ('laser_delay_s', 'gyro_delay_s', 'acceleration_delay_s'):
        if not math.isfinite(getattr(args, name)) or not 0 <= getattr(args, name) <= .15:
            parser.error(f'{name} 必须在 0–0.15 秒')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.pop('ROS_DISCOVERY_SERVER', None)
    workspace = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(workspace/'src/kiwi_slam'))
    os.environ['PYTHONPATH'] = str(workspace/'src/kiwi_slam')+os.pathsep+os.environ.get('PYTHONPATH', '')
    try:
        import numpy as np
        import rclpy
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
        from ament_index_python.packages import get_package_prefix
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry, OccupancyGrid
        from robot_localization.srv import SetPose
        from sensor_msgs.msg import Imu, LaserScan
        from std_msgs.msg import Bool, String
        from tf2_msgs.msg import TFMessage
        from tf2_ros import StaticTransformBroadcaster
    except ImportError as error:
        parser.error(f'请先 source /opt/ros/humble/setup.bash；缺少依赖：{error}')
    output = args.output_dir or Path(tempfile.mkdtemp(prefix='kiwi_recovery_regression_'))
    output.mkdir(parents=True, exist_ok=True)
    children, handles = [], []
    phases, records, truth, checks = [], defaultdict(list), [], []
    thresholds = {'basis': '合成软件回归工程阈值，不是实车精度承诺',
                  'normal_max_scan_slam_gap_s': .65, 'short_recovery_scan_resume_s': .85,
                  'short_max_stamp_gap_s': .25, 'position_residual_base_m': .015,
                  'position_residual_rate_m_s': .35, 'yaw_residual_base_rad': .025,
                  'yaw_residual_rate_rad_s': .30, 'position_truth_p95_m': .15,
                  'yaw_truth_p95_rad': .12}
    result = {'domain_id': args.domain_id, 'hardware': False, 'phases': phases,
              'checks': checks, 'thresholds': thresholds,
              'scenario': {key: value for key, value in vars(args).items() if key != 'output_dir'},
              'truth': {'frame_id': 'odom', 'child_frame_id': 'base_footprint',
                        'time_basis': 'ROS header stamp', 'automatic_alignment': False}}
    source_files = ('tools/test_tracked_recovery.py',
                    'src/kiwi_slam/kiwi_slam/estimation_guard.py',
                    'src/kiwi_slam/kiwi_slam/inertial_odometry.py',
                    'src/kiwi_slam/kiwi_slam/odom_covariance.py',
                    'src/kiwi_slam/kiwi_slam/mapping_gate.py',
                    'src/kiwi_base/config/tracked_pwm.yaml',
                    'src/kiwi_slam/config/tracked_rf2o.yaml',
                    'src/kiwi_slam/config/tracked_mapping_gate.yaml',
                    'src/kiwi_slam/config/tracked_slam_toolbox.yaml')
    def source_hashes():
        return {name: hashlib.sha256((workspace/name).read_bytes()).hexdigest()
                for name in source_files}
    result['source_sha256'] = source_hashes()
    rclpy.init()
    node = Node('tracked_recovery_regression')
    status_qos = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    start = time.monotonic()
    def elapsed():
        return time.monotonic()-start
    def stamp(message):
        return message.header.stamp.sec+message.header.stamp.nanosec*1e-9
    def save_message(name, message):
        row = {'t': elapsed()}
        if isinstance(message, (Bool, String)):
            row['value'] = message.data
        elif isinstance(message, Odometry):
            covariance = np.array(message.pose.covariance).reshape(6, 6)
            row.update(stamp=stamp(message), x=message.pose.pose.position.x,
                       y=message.pose.pose.position.y, yaw=quaternion_yaw(message.pose.pose.orientation),
                       vx=message.twist.twist.linear.x, vy=message.twist.twist.linear.y,
                       wz=message.twist.twist.angular.z, frame_id=message.header.frame_id,
                       child_frame_id=message.child_frame_id,
                       xy_max_variance=float(np.linalg.eigvalsh(covariance[:2, :2]).max()),
                       yaw_variance=float(covariance[5, 5]))
        elif isinstance(message, (LaserScan, OccupancyGrid)):
            row['stamp'] = stamp(message)
        records[name].append(row)
    subs = []
    topics = {
        'odom': (Odometry, '/odometry/filtered', 50),
        'unchecked': (Odometry, '/chassis/odometry/filtered_unchecked', 50),
        'laser': (Odometry, '/chassis/odom/laser', 30),
        'inertial': (Odometry, '/chassis/odom/inertial', 50),
        'scan_slam': (LaserScan, '/scan_slam', qos_profile_sensor_data),
        'map': (OccupancyGrid, '/map', status_qos),
        'gate': (Bool, '/kiwi_slam/mapping_gate_ready', status_qos),
        'valid': (Bool, '/chassis/estimation/valid', status_qos),
        'state': (String, '/chassis/estimation/state', status_qos),
        'imu_ready': (Bool, '/chassis/imu/data_ready', status_qos),
        'inertial_valid': (Bool, '/chassis/inertial/valid', status_qos),
        'inertial_state': (String, '/chassis/inertial/state', status_qos),
        'inertial_reason': (String, '/chassis/inertial/reason', status_qos),
        'laser_ready': (Bool, '/chassis/odom/laser_ready', status_qos),
    }
    for key, (type_, topic, qos) in topics.items():
        subs.append(node.create_subscription(type_, topic,
            lambda message, name=key: save_message(name, message), qos))
    def on_tf(message):
        for transform in message.transforms:
            if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_footprint':
                records['tf'].append({'t': elapsed(), 'stamp': stamp(transform),
                                      'x': transform.transform.translation.x,
                                      'y': transform.transform.translation.y,
                                      'yaw': quaternion_yaw(transform.transform.rotation)})
    subs.append(node.create_subscription(TFMessage, '/tf', on_tf, 100))
    # 透明转发真正的 SetPose 请求并留存时间。只看低频状态可能漏掉快速 reset。
    service_group = ReentrantCallbackGroup()
    real_set_pose = node.create_client(SetPose, '/regression/ekf/set_pose',
                                      callback_group=service_group)
    async def audit_set_pose(request, response):
        pose = request.pose.pose.pose
        entry = {'t': elapsed(), 'stamp': stamp(request.pose),
                 'x': pose.position.x, 'y': pose.position.y,
                 'yaw': quaternion_yaw(pose.orientation)}
        records['set_pose_requests'].append(entry)
        await real_set_pose.call_async(request)
        entry['completed_t'] = elapsed()
        return response
    reset_proxy = node.create_service(SetPose, '/chassis/ekf/set_pose', audit_set_pose,
                                      callback_group=service_group)
    scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    raw_odom = node.create_publisher(Odometry, '/chassis/odom/laser_raw', 10)
    imu_pubs = {name: node.create_publisher(Imu, topic, qos_profile_sensor_data)
                for name, topic in [('raw', '/chassis/imu/data_raw'),
                                    ('gyro', '/chassis/imu/gyro'),
                                    ('acceleration', '/chassis/imu/acceleration')]}
    ready_pubs = [node.create_publisher(Bool, topic, status_qos) for topic in
                  ['/chassis/imu/data_ready', '/chassis/imu/calibration_ready']]
    broadcaster = StaticTransformBroadcaster(node)
    static_tf = TransformStamped()
    static_tf.header.stamp = node.get_clock().now().to_msg()
    static_tf.header.frame_id, static_tf.child_frame_id = 'base_footprint', 'lidar_link'
    static_tf.transform.translation.x = .24
    static_tf.transform.rotation.w = 1.
    broadcaster.sendTransform(static_tf)
    def launch(name, command):
        handle = (output/(name+'.log')).open('w')
        handles.append(handle)
        child = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                 cwd=workspace, start_new_session=True)
        children.append((name, child))
    def source_node(name, script, configuration, extra=()):
        launch(name, [sys.executable, str(workspace/'src/kiwi_slam/scripts'/script),
                      '--ros-args', '--params-file', str(configuration), *extra])
    base_config = workspace/'src/kiwi_base/config/tracked_pwm.yaml'
    slam_config = workspace/'src/kiwi_slam/config'
    x, y, yaw, speed, omega = 0., 0., 0., 0., 0.
    previous, last_scan, last_ready, last_acceleration = elapsed(), -1., -1., -1.
    delayed, serial = [], 0
    def publish(publisher, message, delay):
        nonlocal serial
        if delay == 0:
            publisher.publish(message)
        else:
            serial += 1
            heapq.heappush(delayed, (elapsed()+delay, serial, publisher, message))
    def step(target_speed, scanning, target_omega=0.):
        nonlocal x, y, yaw, speed, omega, previous, last_scan, last_ready, last_acceleration
        for name, child in children:
            if child.poll() is not None:
                raise AssertionError(f'{name} 提前退出 {child.returncode}，见 {output/name}.log')
        now = elapsed()
        dt = now-previous
        acceleration = max(-.24, min(.24, (target_speed-speed)/max(dt, 1e-6)))
        angular_acceleration = max(-.30, min(.30, (target_omega-omega)/max(dt, 1e-6)))
        next_speed, next_omega = speed+acceleration*dt, omega+angular_acceleration*dt
        next_yaw = yaw+(omega+next_omega)*.5*dt
        x += (speed+next_speed)*.5*math.cos((yaw+next_yaw)*.5)*dt
        y += (speed+next_speed)*.5*math.sin((yaw+next_yaw)*.5)*dt
        speed, omega, yaw, previous = next_speed, next_omega, next_yaw, now
        current_stamp = node.get_clock().now().to_msg()
        truth.append({'t': now, 'stamp': current_stamp.sec+current_stamp.nanosec*1e-9,
                      'x': x, 'y': y, 'yaw': yaw, 'vx': speed, 'vy': 0., 'wz': omega})
        raw = Imu()
        raw.header.stamp, raw.header.frame_id = current_stamp, 'chassis_imu_link'
        raw.orientation_covariance[0] = -1.
        raw.angular_velocity.z, raw.angular_velocity_covariance[8] = omega, .001
        raw.linear_acceleration.x, raw.linear_acceleration.y = acceleration, speed*omega
        raw.linear_acceleration.z = 9.80665
        imu_pubs['raw'].publish(raw)
        gyro = Imu()
        gyro.header.stamp, gyro.header.frame_id = current_stamp, 'base_footprint'
        gyro.orientation_covariance[0] = gyro.linear_acceleration_covariance[0] = -1.
        gyro.angular_velocity.z, gyro.angular_velocity_covariance[8] = omega, .001
        publish(imu_pubs['gyro'], gyro, args.gyro_delay_s)
        # 保留实机不同频率；gyro 约 100 Hz，聚合的平面加速度约 30 Hz。
        if now-last_acceleration >= 1./30:
            acc = Imu()
            acc.header.stamp, acc.header.frame_id = current_stamp, 'base_footprint'
            acc.orientation_covariance[0] = acc.angular_velocity_covariance[0] = -1.
            acc.linear_acceleration.x, acc.linear_acceleration.y = acceleration, speed*omega
            acc.linear_acceleration_covariance = [.04, 0., 0., 0., .04, 0., 0., 0., .04]
            publish(imu_pubs['acceleration'], acc, args.acceleration_delay_s)
            last_acceleration = now
        if now-last_ready >= .1:
            for publisher in ready_pubs:
                publisher.publish(Bool(data=True))
            records['ready_input'].append({'t': now, 'value': True})
            last_ready = now
        if scanning and now-last_scan >= 1./12:
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = current_stamp, 'lidar_link'
            scan.angle_min, scan.angle_increment = -math.pi, 2*math.pi/1080
            scan.angle_max = scan.angle_min+1079*scan.angle_increment
            scan.range_min, scan.range_max, scan.scan_time = .15, 12., 1./12
            angles = scan.angle_min+np.arange(1080)*scan.angle_increment+yaw
            dx, dy = np.cos(angles), np.sin(angles)
            laser_x, laser_y = x+.24*math.cos(yaw), y+.24*math.sin(yaw)
            with np.errstate(divide='ignore'):
                tx = np.where(dx > 0, (5.-laser_x)/dx, (-3.-laser_x)/dx)
                ty = np.where(dy > 0, (4.-laser_y)/dy, (-3.-laser_y)/dy)
            scan.ranges = np.minimum(tx, ty).tolist()
            scan_pub.publish(scan)
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id = current_stamp, 'odom'
            odom.child_frame_id = 'base_footprint'
            odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            odom.twist.twist.linear.x, odom.twist.twist.angular.z = speed, omega
            publish(raw_odom, odom, args.laser_delay_s)
            records['scan_input'].append({'t': now, 'stamp': stamp(scan)})
            last_scan = now
        while delayed and delayed[0][0] <= elapsed():
            _, _, publisher, message = heapq.heappop(delayed)
            publisher.publish(message)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.)
        time.sleep(.009)
    def run_phase(name, duration, target_speed, scanning=True):
        begin = elapsed()
        phase = {'name': name, 'begin': begin, 'duration': duration, 'scanning': scanning}
        phases.append(phase)
        print(f'阶段 {name}：{duration:.1f} 秒', flush=True)
        while elapsed()-begin < duration:
            step(target_speed, scanning, args.turn_rate)
        phase.update(end=elapsed(), true_x=x, true_y=y, true_yaw=yaw)
        return phase
    def rows(name, phase, padding=0.):
        return [row for row in records[name] if phase['begin']-padding <= row['t'] <= phase['end']+padding]
    def latest(name):
        return records[name][-1] if records[name] else None
    def check(name, passed, detail):
        checks.append({'name': name, 'passed': bool(passed), 'detail': detail})
    def require(condition, message):
        if not condition:
            raise AssertionError(message)
    def receipt_gap(name, phase):
        times = [phase['begin']]+[row['t'] for row in rows(name, phase)]+[phase['end']]
        return max(b-a for a, b in zip(times, times[1:]))
    return_code = 1
    try:
        until = time.monotonic()+.5
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        occupied = [name for name in node.get_node_names() if name != node.get_name()]
        require(not occupied, f'测试域 {args.domain_id} 已有节点 {occupied}，请选择另一隔离域')
        ekf = Path(get_package_prefix('robot_localization'))/'lib/robot_localization/ekf_node'
        slam = Path(get_package_prefix('slam_toolbox'))/'lib/slam_toolbox/sync_slam_toolbox_node'
        launch('ekf', [str(ekf), '--ros-args', '-r', '__node:=ekf_filter_node',
                      '--params-file', str(base_config),
                      '-r', 'odometry/filtered:=/chassis/odometry/filtered_unchecked',
                      '-r', 'toggle:=/chassis/ekf/toggle', '-r', 'set_pose:=/regression/ekf/set_pose'])
        extra = [] if args.laser_xy_variance is None else ['-p', f'pose_xy_variance:={args.laser_xy_variance}']
        source_node('adapter', 'rf2o_covariance_adapter', slam_config/'tracked_rf2o.yaml', extra)
        source_node('inertial', 'tracked_inertial_odometry', base_config)
        source_node('guard', 'tracked_estimation_guard', base_config)
        source_node('gate', 'kiwi_mapping_scan_gate', slam_config/'tracked_mapping_gate.yaml')
        launch('slam', [str(slam), '--ros-args', '-r', '__node:=slam_toolbox',
                       '--params-file', str(slam_config/'tracked_slam_toolbox.yaml')])
        begin = elapsed()
        while elapsed()-begin < args.startup_timeout:
            step(0., True)
            if (latest('gate') and latest('gate')['value'] and records['map']
                    and latest('state') and latest('state')['value'] == 'VALID'
                    and latest('inertial_valid') and latest('inertial_valid')['value']):
                break
        require(latest('gate') and latest('gate')['value'] and records['map'],
                '启动后建图门未打开或未收到真实 map')
        check('startup_inertial_ready', bool(latest('inertial_valid') and latest('inertial_valid')['value'] and records['inertial']),
              {'state': latest('inertial_state'), 'reason': latest('inertial_reason'), 'count': len(records['inertial'])})
        phases.append({'name': 'startup', 'begin': begin, 'end': elapsed(), 'true_x': x, 'true_y': y, 'true_yaw': yaw})
        normal = run_phase('normal_motion', 3.0, .12)
        forbidden = {'PAUSED', 'PAUSING', 'RESETTING', 'ENABLING', 'WARMING'}
        normal_states = [row for row in rows('state', normal) if row['value'] in forbidden]
        check('normal_start_does_not_reset_or_pause', not normal_states and not rows('set_pose_requests', normal),
              {'bad_states': normal_states, 'set_pose_requests': rows('set_pose_requests', normal)})
        check('normal_motion_keeps_valid', bool(latest('valid') and latest('valid')['value']) and
              not any(not row['value'] for row in rows('valid', normal)), {'last': latest('valid')})
        normal_gap = receipt_gap('scan_slam', normal)
        check('normal_motion_scan_flow', normal_gap <= thresholds['normal_max_scan_slam_gap_s'],
              {'max_receipt_gap_s': normal_gap, 'count': len(rows('scan_slam', normal))})
        short = run_phase('short_loss', .5, .12, False)
        recovered = run_phase('short_recovery', 3.5, .12)
        short_plus = {'begin': short['begin'], 'end': short['end']+.2}
        short_span = {'begin': short['begin'], 'end': recovered['end']}
        check('short_loss_independent_ins_takeover', any(row['value'] == 'INERTIAL' for row in rows('state', short_plus))
              and len(rows('inertial', short)) >= 5,
              {'states': sorted({row['value'] for row in rows('state', short_plus)}), 'inertial_count': len(rows('inertial', short))})
        short_max_gap = {}
        for name in ('odom', 'tf', 'unchecked'):
            frames = rows(name, short, padding=.12)
            gaps = [b['stamp']-a['stamp'] for a, b in zip(frames, frames[1:])]
            short_max_gap[name] = max(gaps) if gaps else None
            check('short_loss_'+name+'_continuous', len(frames) >= 8 and bool(gaps) and
                  max(gaps) < thresholds['short_max_stamp_gap_s'],
                  {'count': len(frames), 'max_stamp_gap_s': short_max_gap[name]})
        check('short_recovery_no_set_pose', not rows('set_pose_requests', short_span),
              {'requests': rows('set_pose_requests', short_span)})
        check('short_recovery_keeps_valid', not any(not row['value'] for row in rows('valid', short_span)) and
              not any(row['value'] in forbidden for row in rows('state', short_span)),
              {'bad_states': [r for r in rows('state', short_span) if r['value'] in forbidden]})
        scan_resumed = rows('scan_slam', recovered)
        resume_latency = scan_resumed[0]['t']-recovered['begin'] if scan_resumed else None
        check('short_recovery_scan_resume_bounded', resume_latency is not None and
              resume_latency <= thresholds['short_recovery_scan_resume_s'] and len(scan_resumed) > 10,
              {'resume_latency_s': resume_latency, 'count': len(scan_resumed)})
        pre_stamp = max(row['stamp'] for row in records['map'] if row['t'] < short['begin'])
        check('short_recovery_map_updates', latest('state')['value'] == 'VALID' and
              any(row['stamp'] > pre_stamp for row in rows('map', recovered)),
              {'last_state': latest('state'), 'map_stamps': [r['stamp'] for r in rows('map', recovered)]})
        long_loss = run_phase('long_loss', 2.7, .12, False)
        tail = {'begin': long_loss['end']-.35, 'end': long_loss['end']}
        check('long_loss_bounded_prediction', any(not row['value'] for row in rows('valid', tail)) and
              any(row['value'] == 'coast_timeout' for row in rows('inertial_reason', tail)) and not rows('odom', tail),
              {'valid': latest('valid'), 'reason': latest('inertial_reason'), 'odom_count': len(rows('odom', tail))})
        pre_stamp = latest('map')['stamp']
        recovered_long = run_phase('long_reference_recovery', 4.0, .12)
        check('long_reference_recovery', latest('state')['value'] == 'VALID' and
              len(rows('scan_slam', recovered_long)) > 10 and
              any(row['stamp'] > pre_stamp for row in rows('map', recovered_long)),
              {'last_state': latest('state'), 'scan_count': len(rows('scan_slam', recovered_long)),
               'map_stamps': [r['stamp'] for r in rows('map', recovered_long)]})
        max_variance = max((row['xy_max_variance'] for row in records['laser']), default=0.)
        if args.laser_xy_variance is not None:
            check('requested_anchor_covariance_exercised', max_variance >= args.laser_xy_variance-1e-6,
                  {'requested_m2': args.laser_xy_variance, 'observed_max_m2': max_variance})
        result['max_laser_xy_variance_m2'] = max_variance
        result['short_loss_max_gap_s'] = short_max_gap
    except Exception as error:
        result['error'] = str(error)
        check('execution_completed', False, str(error))
    finally:
        # 即使先前断言失败，也保存全程轨迹、真值、服务调用与时间戳误差。
        result['source_sha256_at_end'] = source_hashes()
        check('source_version_unchanged_during_run',
              result['source_sha256'] == result['source_sha256_at_end'],
              {'changed': [name for name in source_files
                           if result['source_sha256'][name] != result['source_sha256_at_end'][name]]})
        result['truth_errors'] = {name: truth_errors(records[name], truth)
                                  for name in ('odom', 'unchecked', 'laser', 'inertial')}
        continuity = continuity_metrics(records['odom'], records['valid'])
        result['continuous_valid_pose_twist'] = continuity
        check('continuous_valid_pose_matches_twist', continuity['compared_pairs'] > 20 and
              continuity['violation_count'] == 0,
              {'pairs': continuity['compared_pairs'], 'violation_count': continuity['violation_count'],
               'worst': continuity['largest_events'][:3]})
        truth_metric = result['truth_errors']['odom']
        check('whole_trajectory_synthetic_truth', truth_metric['compared_samples'] > 20 and
              truth_metric['position_m'].get('p95', math.inf) <= thresholds['position_truth_p95_m'] and
              truth_metric['yaw_rad'].get('p95', math.inf) <= thresholds['yaw_truth_p95_rad'], truth_metric)
        state_sequence = []
        for row in records['state']:
            if not state_sequence or state_sequence[-1] != row['value']:
                state_sequence.append(row['value'])
        result.update(passed=bool(checks) and all(check_['passed'] for check_ in checks),
                      message_counts={key: len(value) for key, value in records.items()},
                      state_sequence=state_sequence, true_final_pose={'x': x, 'y': y, 'yaw': yaw},
                      last_values={name: values[-1] for name, values in records.items() if values})
        return_code = 0 if result['passed'] else 1
        for _, child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for _, child in reversed(children):
            try:
                child.wait(timeout=4.)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=2.)
        for handle in handles:
            handle.close()
        (output/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
        (output/'observations.json').write_text(json.dumps(records, ensure_ascii=False)+'\n')
        (output/'truth.json').write_text(json.dumps(truth, ensure_ascii=False)+'\n')
        with (output/'truth.csv').open('w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=['t', 'stamp', 'x', 'y', 'yaw', 'vx', 'vy', 'wz'])
            writer.writeheader()
            writer.writerows(truth)
        print(json.dumps({'passed': result['passed'], 'failed_checks': [r for r in checks if not r['passed']],
                          'state_sequence': state_sequence}, ensure_ascii=False, indent=2), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f'回归证据：{output}', flush=True)
    return return_code


if __name__ == '__main__':
    raise SystemExit(main())
