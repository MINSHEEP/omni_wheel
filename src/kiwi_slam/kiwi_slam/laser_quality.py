"""履带扫描质量：鲁棒背景对齐用于判断运动/退化，不发布第二套里程计。"""
from dataclasses import dataclass
from collections import deque
import math
import numpy as np
from scipy.spatial import cKDTree


def cloud(scan, stride=3):
    ranges = np.asarray(scan.ranges, dtype=float)
    ranges = np.where(np.isfinite(ranges), ranges, np.nan)
    angle = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    p = np.column_stack((ranges * np.cos(angle), ranges * np.sin(angle)))
    # 只在连续表面估计法线，避免将遮挡边缘连接成墙面。
    tangent = np.roll(p, -2, axis=0) - np.roll(p, 2, axis=0)
    length = np.linalg.norm(tangent, axis=1)
    valid = (np.isfinite(p).all(axis=1) & np.isfinite(tangent).all(axis=1)
             & (ranges > scan.range_min) & (ranges < min(scan.range_max, 12.0))
             & (length > 0.015) & (length < 0.5))
    valid[:2] = False
    valid[-2:] = False
    indices = np.flatnonzero(valid)[::stride]
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    return p[indices], normal[indices] / length[indices, None]


@dataclass
class Match:
    pose: np.ndarray
    covariance: np.ndarray
    overlap: float
    residual: float
    condition: float
    valid: bool
    reason: str
    matched: bool = False
    observable_rank: int = 0
    yaw_observability: float = 0.


def align(reference, current, seed=None, min_information_ratio=0.015):
    """点到线 ICP + 截尾/Huber；角度列按场景尺度归一化后检查可观测性。

    covariance 是局部几何形状近似，调用者仍需噪声下限和累计误差传播。
    不把窄走廊的弱方向当作停车证据。
    """
    target, normals = reference
    points, _ = current
    bad = lambda reason: Match(np.zeros(3), np.eye(3)*1e3, 0., math.inf, 0., False, reason)
    if min(len(target), len(points)) < 50:
        return bad('insufficient_points')
    tree = cKDTree(target)
    pose = np.zeros(3) if seed is None else np.asarray(seed, dtype=float).copy()
    for _ in range(12):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        rotated = points @ np.array([[c, s], [-s, c]])
        transformed = rotated + pose[:2]
        distance, idx = tree.query(transformed)
        n = normals[idx]
        residual = np.sum((transformed-target[idx])*n, axis=1)
        valid = (distance < .25) & (np.abs(residual) < .12)
        if np.count_nonzero(valid) < 50:
            return bad('low_overlap')
        cutoff = max(.015, float(np.quantile(np.abs(residual[valid]), .75)))
        valid &= np.abs(residual) <= cutoff
        j = np.column_stack((n[:, 0], n[:, 1], -n[:, 0]*rotated[:, 1]+n[:, 1]*rotated[:, 0]))[valid]
        e = residual[valid]
        weights = np.minimum(1., .02 / np.maximum(np.abs(e), 1e-9))
        h = j.T @ (weights[:, None]*j)
        if not np.isfinite(h).all():
            return bad('invalid_geometry')
        step = np.linalg.lstsq(h, -j.T@(weights*e), rcond=1e-6)[0]
        pose += step
        if np.linalg.norm(step) < 1e-5:
            break
    # 最终位姿重新计算支持率，不能用截尾之后的残差冒充全场景匹配质量。
    c, s = math.cos(pose[2]), math.sin(pose[2])
    rotated = points @ np.array([[c, s], [-s, c]])
    distance, idx = tree.query(rotated+pose[:2])
    n = normals[idx]
    e = np.sum((rotated+pose[:2]-target[idx])*n, axis=1)
    inliers = (distance < .15) & (np.abs(e) < .04)
    overlap = float(np.mean(inliers))
    if np.count_nonzero(inliers) < 50:
        return bad('low_background_support')
    j = np.column_stack((n[:, 0], n[:, 1], -n[:, 0]*rotated[:, 1]+n[:, 1]*rotated[:, 0]))[inliers]
    scale = max(1., float(np.median(np.linalg.norm(points[inliers], axis=1))))
    normalized = j @ np.diag([1., 1., 1./scale])
    eigen, vectors = np.linalg.eigh(normalized.T@normalized / len(j))
    ratio = float(max(0., eigen[0])/max(eigen[-1], 1e-12))
    sigma2 = max(.01**2, float(np.mean(e[inliers]**2)))
    observable = eigen >= max(eigen[-1]*min_information_ratio, 1e-8)
    # pinv 会把零信息方向映为零方差，恰好反转物理含义。弱方向必须高不确定度。
    variance = np.full(3, 1e6)
    variance[observable] = sigma2/eigen[observable]
    units = np.diag([1., 1., 1./scale])
    cov = units@vectors@np.diag(variance)@vectors.T@units
    rank = int(np.count_nonzero(observable))
    yaw_observability = float(np.sum(vectors[2, observable]**2))
    matched = overlap >= .65 and np.linalg.norm(pose[:2]) < 1.0 and abs(pose[2]) < .8
    ok = matched and rank == 3
    reason = 'ok' if ok else ('degenerate' if ratio < min_information_ratio else 'background_mismatch')
    return Match(pose, cov, overlap, float(np.sqrt(sigma2)), ratio, ok, reason,
                 matched, rank, yaw_observability)


class PartialVelocityObservation:
    """仅发布可观测的局部速度信息，不建立或更新全局位姿参考。"""
    def __init__(self, lidar_xy, min_overlap=.8, max_residual=.02,
                 confirmation=.2, max_gap=.3, velocity_floor=(.04, .01, .04)):
        if (not .65 <= min_overlap <= 1. or not 0 < max_residual <= .04
                or not .1 <= confirmation <= 1. or not .1 <= max_gap <= .5):
            raise ValueError('invalid partial laser observation bounds')
        self.lever = np.asarray(lidar_xy, dtype=float)
        self.floor = np.asarray(velocity_floor, dtype=float)
        if (self.lever.shape != (2,) or not np.isfinite(self.lever).all()
                or self.floor.shape != (3,) or not np.isfinite(self.floor).all()
                or np.min(self.floor) <= 0):
            raise ValueError('invalid partial laser geometry or noise floor')
        self.min_overlap, self.max_residual = min_overlap, max_residual
        self.confirmation, self.max_gap = confirmation, max_gap
        self.reset()

    def reset(self):
        self.since = self.last_stamp = None
        self.count = 0
        self.reason = 'waiting_partial_observation'

    def observe(self, match, stamp, reference_stamp):
        dt = None if reference_stamp is None else stamp-reference_stamp
        if (match is None or not math.isfinite(stamp) or dt is None or not 0 < dt <= self.max_gap
                or not match.matched or match.observable_rank < 2
                or match.yaw_observability < .8 or match.overlap < self.min_overlap
                or match.residual > self.max_residual
                or not np.isfinite(match.pose).all() or not np.isfinite(match.covariance).all()):
            self.reset()
            self.reason = 'insufficient_partial_information'
            return None
        # 扫描位移在上一帧雷达系；移除杆臂并旋转到当前 child/base 坐标。
        c, s = math.cos(match.pose[2]), math.sin(match.pose[2])
        rotation = np.array([[c, s], [-s, c]])
        translated = rotation@(match.pose[:2]+self.lever)
        velocity = np.r_[(translated-self.lever)/dt, match.pose[2]/dt]
        jacobian = np.array([[c, s, translated[1]], [-s, c, -translated[0]], [0., 0., 1.]])/dt
        covariance = jacobian@match.covariance@jacobian.T
        # 1e6 仍表示无约束，避免除以很小 dt 后产生无必要的病态量级。
        eigen, vectors = np.linalg.eigh((covariance+covariance.T)*.5)
        covariance = vectors@np.diag(np.clip(eigen, 0., 1e6))@vectors.T+np.diag(self.floor)
        information = np.linalg.pinv(covariance, rcond=1e-10)
        strong = np.linalg.eigh(information)
        projector = strong[1][:, strong[0] > 1e-3]@strong[1][:, strong[0] > 1e-3].T
        observed_velocity = projector@velocity
        if (not np.isfinite(velocity).all() or not np.isfinite(covariance).all()
                or np.linalg.norm(observed_velocity[:2]) > .8 or abs(observed_velocity[2]) > 1.8):
            self.reset()
            self.reason = 'partial_motion_out_of_bounds'
            return None
        if self.last_stamp is None or abs(reference_stamp-self.last_stamp) > 1e-5:
            self.since, self.count = stamp, 0
        self.last_stamp = stamp
        self.count += 1
        if self.count < 3 or stamp-self.since < self.confirmation-1e-8:
            self.reason = 'confirming_partial_observation'
            return None
        self.reason = 'partial_velocity_ready'
        return velocity, covariance


class StationaryAccelerationVeto:
    """已校准加速度只能否决零速，不能建立位置或单独确认平移。"""
    def __init__(self, window=.2, duration=.1, holdoff=.3):
        if not .1 <= window <= .3 or not .05 <= duration <= window or not .1 <= holdoff <= .5:
            raise ValueError('invalid zero velocity acceleration veto timing')
        self.window, self.duration, self.holdoff = window, duration, holdoff
        self.samples = deque()
        self.last_stamp = self.veto_from = self.veto_until = None

    def push(self, stamp, acceleration, covariance):
        acceleration, covariance = np.asarray(acceleration), np.asarray(covariance)
        if (not math.isfinite(stamp) or acceleration.shape != (2,) or covariance.shape != (2, 2)
                or not np.isfinite(acceleration).all() or not np.isfinite(covariance).all()
                or not np.allclose(covariance, covariance.T)
                or np.linalg.eigvalsh(covariance).min() <= 0):
            self.samples.clear()
            return False
        if self.last_stamp is not None and (stamp <= self.last_stamp or stamp-self.last_stamp > .2):
            self.samples.clear()
        self.last_stamp = stamp
        self.samples.append((stamp, acceleration.copy(), covariance.copy()))
        while self.samples and stamp-self.samples[0][0] > self.window+1e-8:
            self.samples.popleft()
        span = stamp-self.samples[0][0]
        if len(self.samples) < 3 or span < self.duration-1e-8:
            return False
        values = np.array([row[1] for row in self.samples])
        mean = values.mean(axis=0)
        # 输入有聚合与相关性，最多视为5个有效样本，且每50ms才增加一份信息；
        # 不让高频重复样本把方差无限压小。窗口内散布另计入检验不确定度。
        effective = min(len(values), 5., 1.+span/.05)
        noise = (np.mean([row[2] for row in self.samples], axis=0)
                 + np.cov(values, rowvar=False, bias=True))/effective
        significant = float(mean @ np.linalg.solve(noise, mean)) > 5.991  # 二维卡方95%
        if significant:
            if self.veto_until is None or stamp > self.veto_until:
                self.veto_from = self.samples[0][0]
            self.veto_until = stamp+self.holdoff
        return significant

    def active(self, stamp):
        return (self.veto_from is not None and self.veto_until is not None
                and self.veto_from <= stamp <= self.veto_until)


class ScanEvidence:
    """固定停车参考防止逐帧零漂；运动参考周期更新，真实慢速运动可累积越过阈值。"""
    def __init__(self, information_ratio=.015, max_gap=2.):
        self.reference = None
        self.reference_time = None
        self.last_time = None
        self.last_match = None
        if not math.isfinite(information_ratio) or not 0 < information_ratio < 1:
            raise ValueError("minimum_information_ratio must be in (0,1)")
        if not .3 < max_gap <= 5:
            raise ValueError("invalid scan recovery gap")
        self.max_gap = max_gap
        self.information_ratio = information_ratio
        self.motion_since = None
        self.static_since = None
        self.stationary = False
        self.healthy = False
        self.reason = 'waiting_scan'
        self.anchor_locked = False
        self.seed = np.zeros(3)
        self.moving_mode = False
        self.matched_reference_time = None
        # 相邻扫描用于判断几何是否恢复；它不能代替丢失的绝对位姿参考。
        self.local_reference = None
        self.local_reference_time = None
        self.local_match = None
        self.local_matched_time = None
        self.local_healthy_since = None
        self.local_healthy_count = 0
        self.local_healthy = False
        self.time_discontinuous = False
        self.last_gyro_rate = math.nan

    def accept_recovered_reference(self, scan, now):
        """仅在调用者完成绝对参考/惯性运动桥核验后切换扫描参考。"""
        points = cloud(scan)
        match = align(points, points, min_information_ratio=self.information_ratio)
        if not match.valid:
            return False
        self.reference, self.reference_time = points, now
        self.matched_reference_time = now
        self.last_match = match
        self.seed = np.zeros(3)
        self.healthy, self.stationary = True, False
        self.anchor_locked = False
        self.moving_mode = True
        self.motion_since = self.static_since = None
        self.time_discontinuous = False
        self.reason = 'reference_recovered'
        # 重置到已核验采样时刻；回调可能已提前缓存之后的扫描，调用者按新参考重放。
        self.last_time = now
        self.local_reference, self.local_reference_time = points, now
        self.local_match = None
        self.local_matched_time = None
        self.local_healthy_since = now
        self.local_healthy_count = 1
        self.local_healthy = False
        return True

    def _observe_local(self, points, now, gyro, dt):
        self.local_matched_time = self.local_reference_time
        if self.local_reference is not None and 0 < dt <= .3 and math.isfinite(gyro):
            self.local_match = align(
                self.local_reference, points, [0., 0., gyro*dt],
                min_information_ratio=self.information_ratio)
        else:
            self.local_match = None
        if self.local_match is not None and self.local_match.valid:
            if self.local_healthy_since is None:
                self.local_healthy_since = now
            self.local_healthy_count += 1
            self.local_healthy = (self.local_healthy_count >= 3
                                  and now-self.local_healthy_since >= .2-1e-8)
        else:
            self.local_healthy_since = None
            self.local_healthy_count = 0
            self.local_healthy = False
        # 这是候选局部参考，即使更新也不代表位姿已被接受。
        self.local_reference, self.local_reference_time = points, now

    def observe(self, scan, now, gyro):
        self.last_gyro_rate = gyro
        points = cloud(scan)
        dt = 0. if self.last_time is None else now-self.last_time
        self._observe_local(points, now, gyro, dt)
        if self.last_time is not None and (now <= self.last_time or now-self.last_time > .3):
            # 断流后无法知道这段时间是否移动；不可无条件换新参考再宣称静止。
            self.healthy = self.stationary = False
            self.reason = ('scan_time_gap_restart_required' if now <= self.last_time
                           else 'scan_gap_requires_reference_validation' if now-self.last_time > self.max_gap
                           else 'scan_interrupted')
            self.time_discontinuous = True
            self.last_time = now
            return
        self.last_time = now
        if self.time_discontinuous:
            # 数据重新连续仅能解除候选几何阻塞，绝对原点需外部核验。
            self.healthy = self.stationary = False
            return
        if self.reference is None:
            initial = align(points, points, min_information_ratio=self.information_ratio)
            if not initial.valid:
                self.last_match = initial
                self.healthy = self.stationary = False
                self.reason = initial.reason
                return
            self.reference, self.reference_time = points, now
        if math.isfinite(gyro):
            self.seed[2] += gyro*dt
        self.matched_reference_time = self.reference_time
        match = align(self.reference, points, seed=self.seed, min_information_ratio=self.information_ratio)
        if match.valid:
            self.seed = match.pose.copy()
        self.last_match = match
        self.healthy = match.valid and math.isfinite(gyro)
        self.reason = match.reason if math.isfinite(gyro) else 'imu_stale'
        if not self.healthy:
            self.stationary = False
            self.static_since = self.motion_since = None
            return
        displacement, angle = np.linalg.norm(match.pose[:2]), abs(match.pose[2])
        moving = displacement > .025 or angle > .02 or abs(gyro) > .025
        still = displacement < .015 and angle < .012 and abs(gyro) < .012
        if not still:
            self.static_since = None
            if self.motion_since is None:
                self.motion_since = now
            if self.moving_mode or now-self.motion_since >= .15 or abs(gyro) > .08:
                self.stationary = False
                self.moving_mode = True
                self.anchor_locked = False
                self.reason = 'moving'
                self.reference, self.reference_time = points, now
                self.seed = np.zeros(3)
                self.motion_since = None
            else:
                # 高质量匹配下的起步确认不是激光失效；最多 .15s 后释放停车保持。
                self.stationary = self.anchor_locked
                self.reason = 'motion_confirmation' if moving else 'ambiguous_motion'
        else:
            self.motion_since = None
            if self.static_since is None:
                self.static_since = now
            self.stationary = self.anchor_locked or now-self.static_since >= 2.
            if self.stationary:
                self.moving_mode = False
                self.anchor_locked = True
                self.reason = 'stationary'
            else:
                self.reason = 'settling'
