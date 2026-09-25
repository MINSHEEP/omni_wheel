"""从静止采集和特征充分的直线启停 bag 生成完整底盘候选 YAML。

不自动覆盖活动配置，不发送电机指令；仅支持 sqlite3 rosbag2。
"""
import argparse
import json
from pathlib import Path
import sqlite3
import time

import numpy as np
import yaml

from .planar_imu import fit_calibration


def read_topic(path, topic, message_type):
    from rclpy.serialization import deserialize_message
    path = Path(path).expanduser().resolve()
    files = sorted(path.glob("*.db3")) if path.is_dir() else [path]
    if not files or any(not p.is_file() for p in files):
        raise ValueError(f"没有 sqlite3 bag：{path}")
    rows = []
    for file in files:
        with sqlite3.connect(file.as_uri() + "?mode=ro", uri=True) as connection:
            item = connection.execute("select id from topics where name=?", (topic,)).fetchone()
            if item is None:
                continue
            for _, payload in connection.execute(
                    "select timestamp,data from messages where topic_id=? order by timestamp", item):
                message = deserialize_message(payload, message_type)
                stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                rows.append((stamp, message))
    if not rows:
        raise ValueError(f"bag 没有话题 {topic}")
    return sorted(rows, key=lambda row: row[0])


def imu_array(rows):
    return np.array([[t, m.linear_acceleration.x, m.linear_acceleration.y,
                      m.linear_acceleration.z, m.angular_velocity.z] for t, m in rows])


def static_samples(rows):
    if len(rows) < 1000 or rows[-1, 0] - rows[0, 0] < 10:
        raise ValueError("静置采样不足 10 秒/1000 帧")
    if not np.isfinite(rows).all() or np.max(np.abs(rows[:, 4])) > 0.08:
        raise ValueError("静置记录包含旋转或非法样本，拒绝标定")
    # 该检查只能拒绝明显运动，不能由 IMU 独自区分静止与匀速。
    if np.max(np.ptp(rows[:, 1:4], axis=0)) > 1.5:
        raise ValueError("静置记录包含冲击/加减速，拒绝标定")
    return rows[:, 1:4]


def forward_windows(imu, odometry):
    """使用直线段 0.9 s 中心差分，避免对 12 Hz 激光速度逐帧求导放大噪声。"""
    motion, reference = [], []
    for t in np.arange(max(imu[0, 0], odometry[0, 0]) + 1,
                       min(imu[-1, 0], odometry[-1, 0]) - 1, 0.15):
        # 整个差分窗口必须无明显转向，避免把转弯/杆臂响应当作前进方向。
        turn = imu[(imu[:, 0] > t - 0.6) & (imu[:, 0] < t + 0.6)]
        center = imu[(imu[:, 0] > t - 0.2) & (imu[:, 0] < t + 0.2)]
        before = odometry[(odometry[:, 0] > t - 0.6) & (odometry[:, 0] < t - 0.3)]
        after = odometry[(odometry[:, 0] > t + 0.3) & (odometry[:, 0] < t + 0.6)]
        if len(center) < 10 or len(before) < 2 or len(after) < 2 or len(turn) < 50:
            continue
        if np.max(np.abs(turn[:, 4])) > 0.07:
            continue
        if max(np.max(np.abs(before[:, 2])), np.max(np.abs(after[:, 2]))) > 0.08:
            continue
        dt = after[:, 0].mean() - before[:, 0].mean()
        acceleration = (after[:, 1].mean() - before[:, 1].mean()) / dt
        if 0.03 < abs(acceleration) < 1.0:
            # 两端速度窗口之差对应加速度的梯形卷积，而非只取中心瞬时加速度。
            # 同一时间核用于两种传感器，避免把起步脉冲误判为比例系数错误。
            weights = np.clip((0.6 - np.abs(turn[:, 0] - t)) / 0.3, 0.0, 1.0)
            motion.append(np.average(turn[:, 1:4], axis=0, weights=weights))
            reference.append(acceleration)
    return np.asarray(motion), np.asarray(reference)


def collect_live(topic, frame, seconds):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    rclpy.init()
    node = rclpy.create_node("planar_imu_calibration")
    rows = []
    def receive(message):
        if message.header.frame_id == frame:
            rows.append((message.header.stamp.sec + message.header.stamp.nanosec * 1e-9, message))
    subscription = node.create_subscription(Imu, topic, receive, qos_profile_sensor_data)
    end = time.monotonic() + seconds
    print(f"采集 {seconds:g} 秒静止 IMU；不发送运动指令。请保持底盘静止。", flush=True)
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    return imu_array(rows)


def main():
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import Odometry
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--static-bag", help="已停止录制且确认为静止的 bag/DB")
    mode.add_argument("--live-static-seconds", type=float, help="人工确认底盘静止后实时采样")
    parser.add_argument("--motion-bag", required=True, help="特征充分区域的直线启停 bag，勿用退化走廊数据")
    parser.add_argument("--config", required=True, help="原有 tracked_pwm.yaml；只读")
    parser.add_argument("--output", required=True, help="新建候选完整底盘 YAML；拒绝覆盖")
    args = parser.parse_args()
    config = yaml.safe_load(Path(args.config).read_text())
    base = config["tracked_base_node"]["ros__parameters"]
    if Path(args.output).exists() or Path(args.output + ".report.json").exists():
        parser.error("输出文件已存在，请选择新路径；不会覆盖活动配置")
    if args.live_static_seconds is not None and not 15 <= args.live_static_seconds <= 1800:
        parser.error("实时采集时长必须为 15～1800 秒")
    def load_imu(path):
        rows = read_topic(path, base["imu_topic"], Imu)
        if any(m.header.frame_id != base["imu_frame_id"] for _, m in rows):
            raise ValueError("bag IMU 坐标与底盘配置不符")
        return imu_array(rows)
    stationary = load_imu(args.static_bag) if args.static_bag else collect_live(
        base["imu_topic"], base["imu_frame_id"], args.live_static_seconds)
    motion = load_imu(args.motion_bag)
    odom = read_topic(args.motion_bag, "/chassis/odom/laser", Odometry)
    if any(m.child_frame_id != "base_footprint" or m.header.frame_id != "odom" for _, m in odom):
        raise ValueError("参考里程计必须为 odom -> base_footprint")
    velocity = np.array([[t, m.twist.twist.linear.x, m.twist.twist.linear.y] for t, m in odom])
    samples, reference = forward_windows(motion, velocity)
    calibration, report = fit_calibration(static_samples(stationary), samples, reference)
    model = config["tracked_robot_model"]["ros__parameters"]
    if any(abs(x) > 1e-8 for x in model["imu_rpy"]):
        raise ValueError("当前标定约定要求模型 imu_rpy 为零；避免重复旋转")
    config["tracked_imu_acceleration"] = {"ros__parameters": {
        "enabled": True, "source_topic": base["imu_topic"], "source_frame": base["imu_frame_id"],
        "output_topic": "/chassis/imu/acceleration", "output_frame": "base_footprint",
        "accel_rotation": calibration.rotation.reshape(-1).tolist(),
        "rest_acceleration": calibration.rest.tolist(),
        "acceleration_variance": calibration.variance.tolist(),
        "imu_lever_xy": model["imu_xyz"][:2],
    }}
    report.update(static_source=args.static_bag or "live", motion_source=args.motion_bag,
                  gyro_z_transform="unchanged in tracked_base_node",
                  recommendation="先离线/手动建图对比；没有独立位置真值，不能据此宣称走廊定位修复")
    with Path(args.output).open("x") as output:
        yaml.safe_dump(config, output, allow_unicode=True, sort_keys=False)
    with Path(args.output + ".report.json").open("x") as output:
        json.dump(report, output, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"候选配置：{args.output}；仅生成文件，尚未替换活动配置")


if __name__ == "__main__":
    main()
