"""Only forward ROS velocity commands while MID360 scan and LIO are fresh."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def sensors_fresh(now, odom_stamp, scan_stamp, max_age):
    if not all(math.isfinite(value) for value in (now, max_age)) or max_age <= 0:
        return False
    return all(stamp is not None and math.isfinite(stamp) and
               -0.1 <= now - stamp <= max_age for stamp in (odom_stamp, scan_stamp))


class Mid360MotionGate(Node):
    def __init__(self):
        super().__init__("mid360_motion_gate")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/mid360/cmd_vel_safe")
        self.declare_parameter("odom_topic", "/fastlio/odom_base")
        self.declare_parameter("scan_topic", "/scan_mid360")
        self.declare_parameter("max_sensor_age_s", 0.5)
        self.declare_parameter("max_cmd_age_s", 0.25)
        def value(name):
            return self.get_parameter(name).value
        self.max_sensor_age = float(value("max_sensor_age_s"))
        self.max_cmd_age = float(value("max_cmd_age_s"))
        if not 0 < self.max_sensor_age <= 2.0 or not 0 < self.max_cmd_age <= 1.0:
            raise ValueError("MID360 速度门控超时必须为有限正数且有上限")
        self.last_odom = None
        self.last_scan = None
        self.last_cmd_time = None
        self.last_cmd = Twist()
        self.publisher = self.create_publisher(Twist, value("output_topic"), 10)
        self.create_subscription(Odometry, value("odom_topic"), self._on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, value("scan_topic"), self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, value("input_topic"), self._on_cmd, 10)
        self.create_timer(0.05, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def _on_odom(self, message):
        self.last_odom = self._stamp(message)

    def _on_scan(self, message):
        self.last_scan = self._stamp(message)

    def _on_cmd(self, message):
        axes = (message.linear.x, message.linear.y, message.linear.z,
                message.angular.x, message.angular.y, message.angular.z)
        if not all(math.isfinite(value) for value in axes):
            self.last_cmd_time = None
            return
        self.last_cmd = message
        self.last_cmd_time = self._now()

    def _tick(self):
        now = self._now()
        if (sensors_fresh(now, self.last_odom, self.last_scan, self.max_sensor_age)
                and self.last_cmd_time is not None
                and 0 <= now - self.last_cmd_time <= self.max_cmd_age):
            self.publisher.publish(self.last_cmd)
        else:
            self.publisher.publish(Twist())


def main():
    rclpy.init()
    node = Mid360MotionGate()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
