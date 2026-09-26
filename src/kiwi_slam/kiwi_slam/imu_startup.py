"""启动静止校准：仅估计残余偏置和噪声，安装轴向/倍率保持不变。"""
from collections import deque
import numpy as np


class StartupCalibration:
    def __init__(self, duration=30., max_gap=.2, recovery_s=.3, max_interruption_s=2.):
        if not 5 <= duration <= 120 or not 0 < max_gap < 1:
            raise ValueError('invalid calibration window')
        if not 0 < recovery_s < max_interruption_s <= 5:
            raise ValueError("invalid IMU recovery bounds")
        self.recovery_s, self.max_interruption_s = recovery_s, max_interruption_s
        self.calibrated = False
        self.recovery_since = None
        self.recovery_count = 0
        self.duration, self.max_gap = duration, max_gap
        self.samples = deque()
        self.last = None
        self.ready = False
        self.failed = False
        self.reason = 'waiting_stationary'
        self.bias = 0.
        self.rest = None
        self.covariance = None

    def push(self, stamp, acceleration, gyro, stationary):
        values = np.asarray([*acceleration, gyro], dtype=float)
        if self.failed:
            return False
        valid = np.isfinite(values).all() and np.isfinite(stamp)
        dt = None if self.last is None else stamp-self.last
        self.last = stamp
        if not valid or (dt is not None and dt <= 0):
            self.invalidate('invalid_or_reversed_imu')
            return False
        if dt is not None and dt > self.max_gap:
            self.samples.clear()
            if self.calibrated and dt > self.max_interruption_s:
                self.invalidate('imu_long_interruption_restart_required')
                return False
            self.pause()
            return False
        if self.calibrated:
            if self.ready:
                return True
            if self.recovery_since is None:
                self.recovery_since = stamp
            self.recovery_count += 1
            # 按采样时间连续观察，积压数据的突发到达不能立即重新放行。
            if stamp-self.recovery_since >= self.recovery_s and self.recovery_count >= 15:
                self.ready = True
                self.reason = 'ready'
            return self.ready
        if not stationary or abs(gyro) > .03 or not 8 < np.linalg.norm(values[:3]) < 11:
            self.samples.clear()
            self.reason = 'waiting_stationary'
            return False
        self.samples.append((stamp, values))
        while len(self.samples)>1 and stamp-self.samples[1][0] > self.duration:
            self.samples.popleft()
        if stamp-self.samples[0][0] < self.duration or len(self.samples) < 50*self.duration:
            self.reason = 'collecting'
            return False
        a = np.asarray([v for _,v in self.samples])
        drift = np.abs(a[:len(a)//2].mean(0)-a[len(a)//2:].mean(0))
        if np.any(a.std(0) > [.15,.15,.15,.015]) or np.any(drift > [.02,.02,.02,.002]):
            self.samples.clear()
            self.reason = 'unstable_window'
            return False
        self.rest = a[:,:3].mean(0)
        self.bias = float(a[:,3].mean())
        self.covariance = np.cov(a, rowvar=False)
        self.ready = self.calibrated = True
        self.reason = 'ready'
        return True

    def pause(self):
        self.ready = False
        self.recovery_since = None
        self.recovery_count = 0
        self.reason = 'recovering_imu' if self.calibrated else 'waiting_stationary'

    def invalidate(self, reason):
        self.ready = self.calibrated = False
        self.failed = True
        self.reason = reason
        self.samples.clear()
