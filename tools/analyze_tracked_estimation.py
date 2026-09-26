#!/usr/bin/env python3
"""只读审查 rosbag2：连续性、源间一致性、外部真值精度分别报告。

source /opt/ros/humble/setup.bash
python3 tools/analyze_tracked_estimation.py BAG --output OUTPUT_PREFIX
CSV 真值必须为 stamp,x,y,yaw；坐标单位 m/rad，与目标话题 frame 完全相同。
时间默认为 ROS epoch 秒；不做自动平移、旋转、尺度或时间对齐。
"""
import argparse
import bisect
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
import zlib

import numpy as np

ODOM = '/odometry/filtered'
LASER = '/chassis/odom/laser'
INS = '/chassis/odom/inertial'
EXPECTED = ['/scan', '/scan_slam', '/chassis/imu/gyro',
            '/chassis/imu/acceleration', LASER, INS, ODOM, '/map',
            '/chassis/estimation/state', '/chassis/inertial/reason']


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))

def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9

def stats(values):
    data = np.asarray(values, dtype=float)
    data = data[np.isfinite(data)]
    if not len(data):
        return {'count': 0}
    return dict(count=len(data), min=float(data.min()), max=float(data.max()),
                mean=float(data.mean()), std=float(data.std()),
                p50=float(np.quantile(data, .5)), p95=float(np.quantile(data, .95)),
                p99=float(np.quantile(data, .99)))

def timing(rows, start):
    received = np.array([r['received'] for r in rows])
    result = {'count': len(rows), 'receipt_gap_s': stats(np.diff(received))}
    if not len(rows):
        return result
    result['recorded_rate_hz'] = ((len(rows)-1)/(received[-1]-received[0])
                                  if received[-1] > received[0] else None)
    stamped = [r for r in rows if 'stamp' in r]
    if not stamped:
        return result
    stamps = np.array([r['stamp'] for r in stamped])
    delta = np.diff(stamps)
    result.update(stamp_gap_s=stats(delta), duplicate_stamps=int((delta == 0).sum()),
                  backward_stamps=int((delta < 0).sum()), unique_stamps=len(set(stamps)),
                  receipt_minus_stamp_s=stats([r['received']-r['stamp'] for r in stamped]))
    largest = np.argsort(delta)[-5:][::-1]
    result['largest_stamp_gaps'] = [dict(
        received_at_s=stamped[int(i)+1]['received']-start,
        previous_stamp=stamped[int(i)]['stamp'], next_stamp=stamped[int(i)+1]['stamp'],
        gap_s=float(delta[i])) for i in largest if delta[i] > 0]
    return result

def pose_row(received, header, child, pose, twist=None, covariance=None):
    result = dict(received=received, stamp=seconds(header.stamp), frame=header.frame_id,
                  child=child, pose=[pose.position.x, pose.position.y, yaw(pose.orientation)])
    if twist is not None:
        result['twist'] = [twist.linear.x, twist.linear.y, twist.angular.z]
    if covariance is not None:
        full = np.asarray(covariance).reshape(6, 6)
        planar = full[np.ix_([0, 1, 5], [0, 1, 5])]
        result['covariance'] = planar.tolist()
        if np.isfinite(planar).all():
            result['xy_variance_max_eigen'] = float(np.linalg.eigvalsh(
                (planar[:2, :2]+planar[:2, :2].T)*.5).max())
            result['covariance_min_eigen'] = float(np.linalg.eigvalsh((planar+planar.T)*.5).min())
            result['yaw_variance'] = float(planar[2, 2])
    return result

def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import yaml
    metadata = yaml.safe_load((path/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=metadata['storage_identifier']),
                rosbag2_py.ConverterOptions('', ''))
    rows = defaultdict(list)
    types, errors, logs = {}, [], []
    declared = {}
    for entry in reader.get_all_topics_and_types():
        declared[entry.name] = entry.type
        try:
            types[entry.name] = get_message(entry.type)
        except (ImportError, AttributeError, ModuleNotFoundError) as exc:
            errors.append(f'{entry.name}: {exc}')
    first = last = None
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        received = timestamp*1e-9
        first = received if first is None else min(first, received)
        last = received if last is None else max(last, received)
        if topic not in types:
            continue
        message = deserialize_message(data, types[topic])
        kind = declared[topic]
        row = {'received': received}
        if kind == 'nav_msgs/msg/Odometry':
            row = pose_row(received, message.header, message.child_frame_id,
                           message.pose.pose, message.twist.twist, message.pose.covariance)
        elif kind == 'tf2_msgs/msg/TFMessage':
            for transform in message.transforms:
                key = topic+':'+transform.header.frame_id+'->'+transform.child_frame_id
                vector = transform.transform.translation
                rows[key].append(dict(received=received, stamp=seconds(transform.header.stamp),
                    frame=transform.header.frame_id, child=transform.child_frame_id,
                    pose=[vector.x, vector.y, yaw(transform.transform.rotation)]))
        elif kind in ('std_msgs/msg/String', 'std_msgs/msg/Bool'):
            row['value'] = message.data
        elif kind == 'sensor_msgs/msg/Imu':
            row.update(stamp=seconds(message.header.stamp), frame=message.header.frame_id,
                acceleration=[message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z],
                angular_velocity=[message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z])
        elif kind == 'sensor_msgs/msg/LaserScan':
            ranges = np.asarray(message.ranges)
            valid = np.isfinite(ranges) & (ranges >= message.range_min) & (ranges <= message.range_max)
            row.update(stamp=seconds(message.header.stamp), frame=message.header.frame_id,
                       valid_ratio=float(valid.mean()) if len(valid) else 0.)
        elif kind == 'nav_msgs/msg/OccupancyGrid':
            row.update(stamp=seconds(message.header.stamp), frame=message.header.frame_id,
                width=message.info.width, height=message.info.height, resolution=message.info.resolution,
                origin=[message.info.origin.position.x, message.info.origin.position.y, yaw(message.info.origin.orientation)],
                crc32=zlib.crc32(np.asarray(message.data, dtype=np.int8).tobytes()))
        elif kind == 'rcl_interfaces/msg/Log':
            logs.append(dict(received=received, stamp=seconds(message.stamp),
                             name=message.name, level=message.level, message=message.msg))
        elif hasattr(message, 'header'):
            row['stamp'] = seconds(message.header.stamp)
        rows[topic].append(row)
    if first is None:
        raise ValueError('bag 没有消息，无法评估')
    return rows, declared, logs, errors, first, last

def state_statistics(rows, start, end):
    intervals = []
    for row in rows:
        value = row['value']
        if intervals and value == intervals[-1]['value']:
            continue
        if intervals:
            intervals[-1]['end_s'] = row['received']-start
        intervals.append(dict(begin_s=row['received']-start, end_s=end-start, value=value))
    durations = defaultdict(float)
    for interval in intervals:
        durations[str(interval['value'])] += interval['end_s']-interval['begin_s']
    return dict(counts=dict(Counter(str(row['value']) for row in rows)),
                duration_s=dict(durations), transitions=max(0, len(intervals)-1), intervals=intervals)

def pose_statistics(rows, start):
    if not rows:
        return {}
    result = {'frames': sorted({r['frame'] for r in rows}),
              'children': sorted({r['child'] for r in rows})}
    positions = np.array([r['pose'] for r in rows])
    result['range_x_m'], result['range_y_m'] = map(float, np.ptp(positions[:, :2], axis=0))
    result['range_yaw_rad'] = float(np.ptp(np.unwrap(positions[:, 2])))
    steps, residuals = [], []
    for previous, current in zip(rows, rows[1:]):
        dt = current['stamp']-previous['stamp']
        if dt <= 0 or previous['frame'] != current['frame'] or previous['child'] != current['child']:
            continue
        displacement = np.asarray(current['pose'][:2])-previous['pose'][:2]
        dyaw = angle(current['pose'][2]-previous['pose'][2])
        step = dict(at_s=current['received']-start, dt_s=dt, distance_m=float(np.linalg.norm(displacement)),
                    yaw_step_rad=dyaw, inferred_speed_m_s=float(np.linalg.norm(displacement)/dt))
        steps.append(step)
        if 'twist' in previous and 'twist' in current:
            velocities = []
            for row in (previous, current):
                c, s = math.cos(row['pose'][2]), math.sin(row['pose'][2])
                vx, vy, _ = row['twist']
                velocities.append(np.array([c*vx-s*vy, s*vx+c*vy]))
            predicted = .5*(velocities[0]+velocities[1])*dt
            yaw_predicted = .5*(previous['twist'][2]+current['twist'][2])*dt
            residuals.append([float(np.linalg.norm(displacement-predicted)),
                              float(np.linalg.norm(displacement/dt-predicted/dt)),
                              abs(angle(dyaw-yaw_predicted)), dt])
    result['largest_pose_steps'] = sorted(steps, key=lambda r:r['distance_m'], reverse=True)[:10]
    result['pose_step_distance_m'] = stats([r['distance_m'] for r in steps])
    result['pose_step_yaw_rad_abs'] = stats([abs(r['yaw_step_rad']) for r in steps])
    if residuals:
        data = np.asarray(residuals)
        result['pose_twist_residual'] = dict(position_m=stats(data[:, 0]), velocity_m_s=stats(data[:, 1]),
            yaw_rad=stats(data[:, 2]), note='相邻位姿差与同段梯形积分Twist的内部一致性；不是外部真值误差')
    for field in ('xy_variance_max_eigen', 'yaw_variance', 'covariance_min_eigen'):
        result[field] = stats([r[field] for r in rows if field in r])
    return result

def interpolate(rows, stamp, max_gap, stamps=None):
    if not rows:
        return None
    stamps = [r['stamp'] for r in rows] if stamps is None else stamps
    index = bisect.bisect_left(stamps, stamp)
    if index < len(rows) and abs(stamps[index]-stamp) < 1e-7:
        return rows[index]['pose']
    if index == 0 or index == len(rows):
        return None
    left, right = rows[index-1], rows[index]
    dt = right['stamp']-left['stamp']
    if not 0 < dt <= max_gap:
        return None
    weight = (stamp-left['stamp'])/dt
    return [left['pose'][i]+weight*(right['pose'][i]-left['pose'][i]) for i in (0, 1)] + [
        angle(left['pose'][2]+weight*angle(right['pose'][2]-left['pose'][2]))]

def compare_sources(primary, other, max_gap):
    if not primary or not other:
        return {'status': 'NOT_EVALUATED', 'reason': '至少一个源没有消息'}
    pairs = {(r['frame'], r['child']) for r in primary+other}
    if len(pairs) != 1:
        return {'status': 'NOT_EVALUATED', 'reason': '源坐标系不一致，不自动对齐'}
    other = sorted(other, key=lambda r:r['stamp'])
    stamps = [r['stamp'] for r in other]
    residuals = []
    for row in primary:
        reference = interpolate(other, row['stamp'], max_gap, stamps)
        if reference is not None:
            residuals.append([math.hypot(row['pose'][0]-reference[0], row['pose'][1]-reference[1]),
                              abs(angle(row['pose'][2]-reference[2]))])
    return dict(status='EVALUATED' if residuals else 'NOT_EVALUATED',
        matched_samples=len(residuals), position_m=stats([r[0] for r in residuals]),
        yaw_rad=stats([r[1] for r in residuals]),
        note='按原始ROS采样时间插值；仅源间一致性，同源传感器的一致不证明位置准确')

def accuracy(rows, args, start):
    if args.groundtruth is None:
        return {'status': 'NOT_EVALUATED', 'reason': '没有独立外部真值；连续性或源间一致性不能替代精度'}
    if not args.truth_frame:
        raise ValueError('提供真值时必须显式传入 --truth-frame，工具不猜测坐标系')
    if not rows or {r['frame'] for r in rows} != {args.truth_frame}:
        return {'status': 'NOT_EVALUATED', 'reason': '目标消息缺失或目标frame与真值frame不一致'}
    truth = []
    with args.groundtruth.open(newline='') as stream:
        reader = csv.DictReader(stream)
        if not {'stamp', 'x', 'y', 'yaw'} <= set(reader.fieldnames or []):
            raise ValueError('真值CSV必须包含stamp,x,y,yaw')
        for entry in reader:
            values = [float(entry[key]) for key in ('stamp', 'x', 'y', 'yaw')]
            if not all(math.isfinite(value) for value in values):
                raise ValueError('真值CSV含非有限数值')
            stamp = values[0]+args.truth_time_offset_s+(start if args.truth_time_basis == 'bag-relative' else 0.)
            if truth and stamp <= truth[-1]['stamp']:
                raise ValueError('真值时间必须严格递增，不排序修补重复或倒序记录')
            truth.append(dict(stamp=stamp, pose=values[1:]))
    errors = []
    stamps = [r['stamp'] for r in truth]
    for row in rows:
        reference = interpolate(truth, row['stamp'], args.truth_max_gap_s, stamps)
        if reference is not None:
            errors.append([math.hypot(row['pose'][0]-reference[0], row['pose'][1]-reference[1]),
                           abs(angle(row['pose'][2]-reference[2]))])
    return dict(status='EVALUATED' if errors else 'NOT_EVALUATED', frame=args.truth_frame,
        source=str(args.groundtruth.resolve()), source_sha256=hashlib.sha256(args.groundtruth.read_bytes()).hexdigest(),
        matched_samples=len(errors), unmatched_samples=len(rows)-len(errors),
        rmse_position_m=float(np.sqrt(np.mean(np.square([e[0] for e in errors])))) if errors else None,
        rmse_yaw_rad=float(np.sqrt(np.mean(np.square([e[1] for e in errors])))) if errors else None,
        position_m=stats([e[0] for e in errors]), yaw_rad=stats([e[1] for e in errors]),
        time_basis=args.truth_time_basis, explicit_time_offset_s=args.truth_time_offset_s,
        interpolation_max_gap_s=args.truth_max_gap_s, alignment='NONE：不优化平移、旋转、尺度或时间偏移',
        verdict='NOT_SCORED：报告误差，没有由工具自选的验收门槛')

def build_report(rows, declared, logs, errors, start, end, args):
    report = dict(bag=str(args.bag.resolve()), start_stamp=start, end_stamp=end, duration_s=end-start,
        timing_convention='bag接收时间减header采样时间；不同设备时钟偏移、SLAM的未来TF stamp需单独解释',
        read_errors=errors, topics={}, states={}, poses={}, source_consistency={})
    for topic in sorted(set(rows)|set(declared)|set(EXPECTED)):
        values = rows[topic]
        report['topics'][topic] = timing(values, start)
        if values and 'value' in values[0]:
            report['states'][topic] = state_statistics(values, start, end)
        if values and 'pose' in values[0]:
            report['poses'][topic] = pose_statistics(values, start)
        if values and 'acceleration' in values[0]:
            report['topics'][topic]['imu_statistics'] = {
                key: {axis:stats([r[key][i] for r in values]) for i,axis in enumerate('xyz')}
                for key in ('acceleration', 'angular_velocity')}
    for name, other in [('public_vs_laser', LASER), ('public_vs_ins', INS),
                        ('public_vs_unchecked', '/chassis/odometry/filtered_unchecked'),
                        ('public_vs_tf', '/tf:odom->base_footprint')]:
        report['source_consistency'][name] = compare_sources(rows[ODOM], rows[other], args.max_interpolation_gap_s)
    maps = rows['/map']
    report['map'] = dict(count=len(maps), unique_stamps=len({r['stamp'] for r in maps}),
        distinct_grids=len({(r['width'], r['height'],r['resolution'],tuple(r['origin']),r['crc32']) for r in maps}),
        snapshots=[dict(at_s=r['received']-start, **{k:v for k,v in r.items() if k != 'received'}) for r in maps],
        note='地图发布/内容变化与扫描帧率不同；静止关键帧不变本身不证明故障')
    warning_counts = Counter((row['name'], row['message']) for row in logs if row['level'] >= 30)
    report['warnings'] = [dict(node=key[0], message=key[1], count=count) for key,count in warning_counts.most_common(50)]
    report['external_accuracy'] = accuracy(rows[args.truth_topic], args, start)
    report['launch_logs'] = []
    for path in args.launch_log:
        contents = path.read_text(errors='replace')
        lines = contents.splitlines()
        report['launch_logs'].append(dict(path=str(path.resolve()),
            sha256=hashlib.sha256(path.read_bytes()).hexdigest(), lines=len(lines),
            process_start_lines=sum('process started' in row for row in lines),
            warning_error_lines=[row for row in lines if any(marker in row for marker in
                ('[WARN]', '[ERROR]', '[FATAL]', 'process has died', 'Traceback'))][:50],
            note='只有launch进程启动行时不能判断后续节点状态；同时参考录包rosout'))
    report['continuity'] = dict(status='MEASURED', note='时序统计，不等于定位精度PASS；启动录包发现延迟/缓存首帧需单独判断')
    return report

def number(value):
    return '—' if value is None else f'{value:.6f}'

def markdown(report):
    lines = ['# 履带估计录包审查', '', f"数据：`{report['bag']}`",
             f"录制窗口：{report['duration_s']:.3f} 秒。外部精度：**{report['external_accuracy']['status']}**。",
             '', '连续性、源间一致性、外部真值精度分别评估；没有真值不能宣布精度通过。',
             '', '## 时序与延迟', '',
             '接收延迟=bag接收时间−消息header采样时间；负值可能来自未来TF时间戳，不能直接判定消息顺序错误。', '',
             '| 话题/TF | 帧数 | 接收Hz | 最大stamp间隔s | 延迟p95 s | 倒序stamp |',
             '| --- | ---: | ---: | ---: | ---: | ---: |']
    for topic,data in report['topics'].items():
        if topic in EXPECTED or topic.startswith('/tf:') or 'stamp_gap_s' in data:
            lines.append(f"| `{topic}` | {data['count']} | {number(data.get('recorded_rate_hz'))} | {number(data.get('stamp_gap_s',{}).get('max'))} | {number(data.get('receipt_minus_stamp_s',{}).get('p95'))} | {data.get('backward_stamps','—')} |")
    lines += ['', '## 状态与开关', '']
    for topic,data in report['states'].items():
        lines += [f"- `{topic}`：{json.dumps(data['counts'],ensure_ascii=False)}；切换 {data['transitions']} 次。"]
        if data['transitions']:
            lines += [f"  - {row['begin_s']:.3f}–{row['end_s']:.3f} s：`{row['value']}`" for row in data['intervals'][:100]]
    lines += ['', '## 位姿、速度与协方差', '', '| 话题 | x/y范围m | 最大xy方差m² | 位姿-速度残差p95 m/s |', '| --- | --- | ---: | ---: |']
    for topic,data in report['poses'].items():
        lines.append(f"| `{topic}` | {number(data['range_x_m'])}/{number(data['range_y_m'])} | {number(data['xy_variance_max_eigen'].get('max'))} | {number(data.get('pose_twist_residual',{}).get('velocity_m_s',{}).get('p95'))} |")
    lines += ['', '## 源间一致性', '', '按原始采样时间插值，同源IMU/激光产生的多个输出相互接近不构成外部精度证明。', '']
    for name,data in report['source_consistency'].items():
        lines.append(f"- {name}：{data['status']}，样本 {data.get('matched_samples',0)}，位置差p95 {number(data.get('position_m',{}).get('p95'))} m。")
    lines += ['', '## 地图更新', '', f"收到 {report['map']['count']} 条地图，{report['map']['unique_stamps']} 个不同时间戳，{report['map']['distinct_grids']} 种栅格/边界内容。地图更新频率不等于激光扫描频率。",
              '', '## 外部真值精度', '', '```json',json.dumps(report['external_accuracy'],ensure_ascii=False,indent=2), '```',
              '', '## 警告日志', '']
    lines += [f"- {item['node']}（{item['count']} 次）：{item['message']}" for item in report['warnings']] or ['本录包未记录 WARN/ERROR。']
    if report['launch_logs']:
        lines += ['', '## 启动日志出处', '']
        lines += [f"- `{item['path']}`：{item['lines']} 行，进程启动 {item['process_start_lines']} 行；SHA256 `{item['sha256']}`。" for item in report['launch_logs']]
        lines += ['', '启动记录相同不证明节点运行期间没有错误；故障结论须结合采集时段与 rosout。']
    return '\n'.join(lines)+'\n'

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag', type=Path, help='包含metadata.yaml的rosbag2目录')
    parser.add_argument('--output', type=Path, help='输出前缀；生成.json和.md，默认bag同级estimation_review')
    parser.add_argument('--max-interpolation-gap-s', type=float, default=.2)
    parser.add_argument('--launch-log', type=Path, action='append', default=[], help='可重复指定，记录启动日志哈希及警告')
    parser.add_argument('--groundtruth', type=Path, help='独立外部真值CSV：stamp,x,y,yaw，单位s/m/rad')
    parser.add_argument('--truth-topic', default=ODOM)
    parser.add_argument('--truth-frame', help='CSV的xy/yaw所属坐标系，必须与目标topic一致')
    parser.add_argument('--truth-time-basis', choices=['ros','bag-relative'], default='ros')
    parser.add_argument('--truth-time-offset-s', type=float, default=0., help='显式已知偏移：真值stamp+offset=ROS时间；不自动拟合')
    parser.add_argument('--truth-max-gap-s', type=float, default=.2)
    args = parser.parse_args()
    if any(not math.isfinite(v) or v <= 0 for v in (args.max_interpolation_gap_s, args.truth_max_gap_s)):
        parser.error('插值间隔必须为有限正数')
    if not math.isfinite(args.truth_time_offset_s):
        parser.error('真值时间偏移必须为有限数值')
    if not (args.bag/'metadata.yaml').is_file():
        parser.error('bag目录内没有metadata.yaml')
    rows, declared, logs, errors, start, end = read_bag(args.bag)
    report = build_report(rows, declared, logs, errors, start, end, args)
    output = args.output or args.bag.parent/'estimation_review'
    output.parent.mkdir(parents=True, exist_ok=True)
    json_path, md_path = Path(str(output)+'.json'), Path(str(output)+'.md')
    json_path.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    md_path.write_text(markdown(report))
    print(json.dumps(dict(json=str(json_path), markdown=str(md_path), duration_s=report['duration_s'],
                         external_accuracy=report['external_accuracy']['status']),ensure_ascii=False))

if __name__ == '__main__':
    main()
