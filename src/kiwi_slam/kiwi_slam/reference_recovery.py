"""核验旧扫描或有界惯性运动桥，恢复时不悄悄重置里程计原点。"""
import math
import numpy as np
from .laser_quality import cloud, align


def relative_pose(previous, current):
    c, s = math.cos(previous[2]), math.sin(previous[2])
    delta = np.asarray(current[:2])-np.asarray(previous[:2])
    return np.r_[np.array([[c, s], [-s, c]]) @ delta,
                 math.remainder(current[2]-previous[2], 2*math.pi)]


def compose_pose(previous, relative):
    c, s = math.cos(previous[2]), math.sin(previous[2])
    return np.r_[np.asarray(previous[:2])+np.array([[c, -s], [s, c]])@relative[:2],
                 math.remainder(previous[2]+relative[2], 2*math.pi)]


class ReferenceRecovery:
    def __init__(self, lidar_xy, max_gap=2., duration=.2, reference_max_gap=None,
                 max_xy_variance=4., max_yaw_variance=.15,
                 max_xy_growth=.09, max_yaw_growth=.05):
        reference_max_gap = max_gap if reference_max_gap is None else reference_max_gap
        if not 0 < duration < max_gap <= 5 or not max_gap <= reference_max_gap <= 30:
            raise ValueError('invalid reference recovery bounds')
        if not np.isfinite([max_xy_variance, max_yaw_variance]).all() or min(max_xy_variance, max_yaw_variance) <= 0:
            raise ValueError('invalid inertial recovery covariance limits')
        self.lever = np.asarray(lidar_xy, dtype=float)
        self.max_gap, self.duration = max_gap, duration
        self.reference_max_gap = reference_max_gap
        self.max_xy_variance, self.max_yaw_variance = max_xy_variance, max_yaw_variance
        if not np.isfinite([max_xy_growth, max_yaw_growth]).all() or min(max_xy_growth, max_yaw_growth) <= 0:
            raise ValueError('invalid inertial recovery growth limits')
        self.max_xy_growth, self.max_yaw_growth = max_xy_growth, max_yaw_growth
        self.reference = None
        self.corrected_reference = None
        self.reference_covariance = np.zeros((3, 3))
        self.reference_full_covariance = None
        self.recovered_pose = None
        self.recovered_covariance = None
        self.since = None
        self.count = 0
        self.reason = 'waiting_reference'
        self.method = None
        self.previous_candidate = None

    def accept(self, scan, stamp, raw, corrected=None, covariance=None, baseline_covariance=None):
        self.reference = (cloud(scan), stamp, np.asarray(raw).copy())
        self.corrected_reference = np.asarray(raw if corrected is None else corrected).copy()
        self.reference_covariance = (np.zeros((3, 3)) if covariance is None
                                     else np.asarray(covariance, dtype=float).copy())
        # covariance 是累计原点误差；baseline 是 INS 真正继承的完整测量协方差。
        self.reference_full_covariance = (None if baseline_covariance is None
                                         else np.asarray(baseline_covariance, dtype=float).copy())
        self.since = self.previous_candidate = self.method = None
        self.count = 0

    def _reject(self, reason):
        self.since = self.previous_candidate = self.method = None
        self.count = 0
        self.reason = reason
        self.recovered_pose = self.recovered_covariance = None
        return False

    def _base_delta(self, laser_pose):
        c, s = math.cos(laser_pose[2]), math.sin(laser_pose[2])
        return np.r_[laser_pose[:2]+self.lever-np.array([[c, -s], [s, c]])@self.lever, laser_pose[2]]

    def _candidate(self, stamp, pose, covariance, method, long_reference=False):
        if self.previous_candidate is not None and self.method == method:
            old_stamp, old_pose = self.previous_candidate
            dt = stamp-old_stamp
            delta = relative_pose(old_pose, pose)
            if not 0 < dt <= .3 or np.linalg.norm(delta[:2]) > .7*dt+.03 or abs(delta[2]) > 1.5*dt+.03:
                return self._reject('recovery_motion_inconsistent')
        else:
            self.since, self.count = None, 0
        if self.since is None:
            self.since = stamp
        self.method = method
        self.previous_candidate = (stamp, np.asarray(pose).copy())
        self.count += 1
        ready = (self.count >= (5 if long_reference else 3)
                 and stamp-self.since >= (max(.4, self.duration) if long_reference else self.duration)-1e-8)
        self.recovered_pose = np.asarray(pose).copy() if ready else None
        self.recovered_covariance = np.asarray(covariance).copy() if ready else None
        self.reason = method+'_verified' if ready else 'verifying_'+method
        return ready

    def _valid_inertial(self, sample, stamp):
        if sample is None or abs(sample['stamp']-stamp) > .15:
            return False
        pose, covariance = np.asarray(sample['pose']), np.asarray(sample['covariance'])
        if not (pose.shape == (3,) and covariance.shape == (3, 3)
                and np.isfinite(pose).all() and np.isfinite(covariance).all()
                and np.linalg.eigvalsh(covariance).min() >= -1e-9
                and min(covariance[0, 0], covariance[1, 1], covariance[2, 2]) > 0):
            return False
        xy_limit, yaw_limit = self.max_xy_variance, self.max_yaw_variance
        baseline = self.reference_full_covariance
        if baseline is not None:
            inherited_xy = np.linalg.eigvalsh(baseline[:2, :2]).max()
            if inherited_xy > xy_limit:
                xy_limit = inherited_xy+self.max_xy_growth
            if baseline[2, 2] > yaw_limit:
                yaw_limit = baseline[2, 2]+self.max_yaw_growth
            if (np.linalg.eigvalsh(covariance[:2, :2]-baseline[:2, :2]).max() > self.max_xy_growth
                    or covariance[2, 2]-baseline[2, 2] > self.max_yaw_growth):
                return False
        return (np.linalg.eigvalsh(covariance[:2, :2]).max() <= xy_limit
                and covariance[2, 2] <= yaw_limit)

    def check(self, scan, stamp, raw, stationary, information_ratio, *,
              inertial=None, local_match=None, local_inertial=None,
              local_stamp=None, local_healthy=False):
        if self.reference is None:
            return self._reject('waiting_reference')
        points, previous_stamp, previous = self.reference
        dt = stamp-previous_stamp
        if not 0 < dt <= self.reference_max_gap:
            return self._reject('reference_expired_restart_required')
        delta = relative_pose(previous, raw)
        c, s = math.cos(delta[2]), math.sin(delta[2])
        laser_delta = np.r_[delta[:2]+np.array([[c, -s], [s, c]])@self.lever-self.lever, delta[2]]
        # 旧参考仍能匹配时可找回同一原点；长间隔采用更严格的几何和连续帧核验。
        match = align(points, cloud(scan), laser_delta, information_ratio)
        if not match.valid:
            alternative = align(points, cloud(scan), np.zeros(3), information_ratio)
            if alternative.valid:
                match = alternative
        base_delta = self._base_delta(match.pose)
        consistent = (np.linalg.norm(delta[:2]-base_delta[:2]) < .08
                      and abs(math.remainder(delta[2]-base_delta[2], 2*math.pi)) < .08)
        long_reference = dt > self.max_gap
        strong = (match.overlap >= .8 and match.residual <= .02
                  and match.condition >= 2*information_ratio)
        require_strong = long_reference or not consistent
        if require_strong and match.valid and strong:
            independent = align(points, cloud(scan), match.pose+np.array([.04, -.04, .04]), information_ratio)
            strong = (independent.valid and independent.overlap >= .8
                      and independent.residual <= .02
                      and independent.condition >= 2*information_ratio
                      and np.linalg.norm(independent.pose[:2]-match.pose[:2]) < .03
                      and abs(math.remainder(independent.pose[2]-match.pose[2], 2*math.pi)) < .03)
        # RF2O 自身跳变时需扰动初值下的强几何一致性；stale stationary 不能把位移归零。
        if match.valid and ((consistent and not long_reference) or strong):
            corrected = compose_pose(self.corrected_reference, base_delta)
            c, s = math.cos(self.corrected_reference[2]), math.sin(self.corrected_reference[2])
            transform = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
            lever_jacobian = np.array([[1., 0., self.lever[1]], [0., 1., -self.lever[0]], [0., 0., 1.]])
            covariance = self.reference_covariance + transform@lever_jacobian@match.covariance@lever_jacobian.T@transform.T
            return self._candidate(stamp, corrected, covariance, 'reference', require_strong)
        if (dt > self.max_gap or not local_healthy or local_match is None
                or not local_match.valid or local_stamp is None
                or not self._valid_inertial(inertial, stamp)
                or not self._valid_inertial(local_inertial, local_stamp)):
            return self._reject('reference_mismatch')
        # 独立惯导保留原 odom 坐标；连续局部扫描只核验它的相对运动。
        inertial_delta = relative_pose(local_inertial['pose'], inertial['pose'])
        observed_delta = self._base_delta(local_match.pose)
        if (np.linalg.norm(inertial_delta[:2]-observed_delta[:2]) > .06
                or abs(math.remainder(inertial_delta[2]-observed_delta[2], 2*math.pi)) > .08):
            return self._reject('inertial_scan_disagreement')
        total_delta = relative_pose(self.corrected_reference, inertial['pose'])
        if np.linalg.norm(total_delta[:2]) > .7*dt+.03 or abs(total_delta[2]) > 1.5*dt+.03:
            return self._reject('inertial_bridge_out_of_bounds')
        return self._candidate(stamp, inertial['pose'], inertial['covariance'], 'inertial_bridge')
