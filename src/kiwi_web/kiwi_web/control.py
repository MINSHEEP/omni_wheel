"""Velocity arbitration independent of HTTP and ROS; all deadlines are monotonic."""

import math
import secrets
import threading
import time


class CommandGate:
    def __init__(
        self, linear=0.4, lateral=0.4, angular=0.6, clock=time.monotonic, navigation_limits=None
    ):
        self.lock = threading.RLock()
        self.clock = clock
        self.limits = (linear, lateral, angular)
        self.navigation_limits = navigation_limits or self.limits
        self.owner = None
        self.lease_until = 0.0
        self.motion_until = 0.0
        self.velocity = (0.0, 0.0, 0.0)
        self.estop = False
        self.mode = "idle"
        self.manual_ready = False
        self.nav_ready = False
        self.nav_enabled = False
        self.nav_velocity = (0.0, 0.0, 0.0)
        self.nav_until = 0.0
        self.challenges = {}
        self.sequence = -1

    def challenge(self):
        with self.lock:
            now = self.clock()
            self.challenges = {k: t for k, t in self.challenges.items() if t > now}
            token = secrets.token_hex(8)
            self.challenges[token] = now + 0.3
            return token

    def acquire(self, owner):
        with self.lock:
            self._expire()
            if self.owner and self.owner != owner:
                raise ValueError("另一浏览器正在遥控，请先释放控制权")
            if self.estop or self.mode not in ("mapping", "remote") or not self.manual_ready:
                raise ValueError("底盘未就绪、模式不支持遥控或软件急停未解除")
            self.owner = owner
            self.lease_until = self.clock() + 1.0
            self.sequence = -1
            self.nav_enabled = False

    def drive(self, owner, velocity, sequence, challenge):
        with self.lock:
            self._expire()
            if owner != self.owner or self.owner is None:
                raise ValueError("没有遥控权，请重新开启遥控")
            if self.challenges.get(challenge, 0) <= self.clock():
                raise ValueError("控制报文已过期，已拒绝延迟指令")
            if type(sequence) is not int or sequence <= self.sequence:
                raise ValueError("控制报文序号无效")
            self.velocity = self._bounded(velocity)
            self.sequence = sequence
            self.lease_until = self.clock() + 1.0
            self.motion_until = self.clock() + 0.25

    def _bounded(self, values, limits=None):
        if len(values) != 3 or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
            for v in values
        ):
            raise ValueError("速度必须是三个有限数值")
        active_limits = limits or self.limits
        if any(limit == 0 and value != 0 for value, limit in zip(values, active_limits)):
            raise ValueError("当前底盘不支持该方向的运动")
        ratio = max(1.0, *(abs(v) / limit for v, limit in zip(values, active_limits) if limit > 0))
        return tuple(v / ratio for v in values)

    def release(self, owner=None):
        with self.lock:
            if owner is None or self.owner == owner:
                self.owner = None
                self.velocity = (0.0, 0.0, 0.0)
                self.motion_until = 0.0

    def stop(self, latch=False):
        with self.lock:
            self.release()
            self.nav_enabled = False
            self.nav_until = 0.0
            self.estop = self.estop or latch

    def reset(self):
        with self.lock:
            self.stop()
            self.estop = False

    def navigation(self, values):
        with self.lock:
            self.nav_velocity = self._bounded(values, self.navigation_limits)
            self.nav_until = self.clock() + 0.25

    def _expire(self):
        if self.clock() >= self.lease_until:
            self.release()

    def output(self):
        with self.lock:
            self._expire()
            if not self.estop:
                if self.owner and self.manual_ready and self.clock() < self.motion_until:
                    return self.velocity
                if (
                    self.mode == "navigation"
                    and self.nav_enabled
                    and self.nav_ready
                    and not self.owner
                    and self.clock() < self.nav_until
                ):
                    return self.nav_velocity
            return (0.0, 0.0, 0.0)

    def status(self):
        with self.lock:
            self._expire()
            return dict(
                owner=self.owner,
                estop=self.estop,
                manual_ready=self.manual_ready,
                nav_ready=self.nav_ready,
                limits=self.limits,
            )
