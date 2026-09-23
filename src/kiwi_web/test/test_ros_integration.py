"""Actual DDS/action integration in an isolated localhost ROS domain."""

import asyncio
import json
import os
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from kiwi_web.control import CommandGate
from kiwi_web.ros_bridge import Navigator, RosBridge


async def until(predicate, timeout=8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        await asyncio.sleep(0.02)
    raise AssertionError("ROS condition timed out")


class FixtureRobot(Node):
    def __init__(self, width=4, height=3, data=None, tracked=False):
        super().__init__("web_test_robot")
        self.tracked = tracked
        self.group = ReentrantCallbackGroup()
        self.healthy = True
        self.observed = []
        self.goals = []
        self.canceled = 0
        self.grid_width, self.grid_height = width, height
        self.grid_data = data if data is not None else [0, 0, 100, -1] * 3
        self.last_map = 0.0
        self.feedback_pub = self.create_publisher(
            Odometry, "/chassis/odom/laser" if tracked else "/chassis/odom/raw", 1
        )
        self.odom_pub = self.create_publisher(Odometry, "/odometry/filtered", 1)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.imu_pub = self.create_publisher(
            Imu if tracked else Bool,
            "/chassis/imu/data_raw" if tracked else "/chassis/imu/calibration_ready",
            1 if tracked else latched,
        )
        self.gate_pub = self.create_publisher(Bool, "/kiwi_slam/mapping_gate_ready", latched)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", latched)
        self.nav_pub = self.create_publisher(Twist, "/kiwi_web/nav_cmd_vel", 1)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, "/cmd_vel", self.observe, 10, callback_group=self.group)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", lambda _: None, 1)
        self.create_service(GetState, "/bt_navigator/get_state", self.lifecycle_state)
        self.server = ActionServer(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            execute_callback=self.execute,
            goal_callback=self.accept,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.group,
        )
        self.create_timer(0.04, self.publish, callback_group=self.group)

    def lifecycle_state(self, request, response):
        response.current_state.id = 3
        return response

    def observe(self, message):
        self.observed.append(
            (time.monotonic(), message.linear.x, message.linear.y, message.angular.z)
        )
        self.observed = self.observed[-200:]

    def publish(self):
        if not self.healthy:
            return
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.orientation.w = 1.0
        self.feedback_pub.publish(odom)
        self.odom_pub.publish(odom)
        if self.tracked:
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = stamp, "chassis_imu_link"
            self.imu_pub.publish(imu)
        else:
            self.imu_pub.publish(Bool(data=True))
        self.gate_pub.publish(Bool(data=True))
        transforms = []
        for child in ("base_footprint", "lidar_link"):
            transform = TransformStamped()
            transform.header.stamp, transform.header.frame_id = stamp, "map"
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self.tf.sendTransform(transforms)
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, "lidar_link"
        scan.angle_increment, scan.range_min, scan.range_max = 0.1, 0.05, 20.0
        scan.ranges = [1.0] * 20
        self.scan_pub.publish(scan)
        grid = OccupancyGrid()
        grid.header.stamp, grid.header.frame_id = stamp, "map"
        grid.info.width, grid.info.height, grid.info.resolution = (
            self.grid_width,
            self.grid_height,
            0.1,
        )
        grid.info.origin.orientation.w = 1.0
        grid.data = self.grid_data
        if time.monotonic() - self.last_map > 0.5:
            self.map_pub.publish(grid)
            self.last_map = time.monotonic()
        velocity = Twist()
        velocity.linear.x = 0.15
        self.nav_pub.publish(velocity)

    def accept(self, request):
        if request.pose.pose.position.x == 77:
            time.sleep(0.2)
        return GoalResponse.ACCEPT

    def execute(self, handle):
        self.goals.append(handle.request.pose.pose.position.x)
        duration = 8 if handle.request.pose.pose.position.x in (77, 99) else 0.25
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if handle.is_cancel_requested:
                self.canceled += 1
                handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.01)
        handle.succeed()
        return NavigateToPose.Result()


def test_real_ros_navigation_pause_resume_cancel_and_feedback_loss():
    os.environ["ROS_DOMAIN_ID"] = "216"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init()
    gate = CommandGate()
    gate.mode = "navigation"
    bridge, robot = RosBridge(gate), FixtureRobot()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    executor.add_node(robot)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    async def exercise():
        navigator = Navigator(bridge, gate)
        await until(lambda: gate.nav_ready and bridge.navigation.server_is_ready())
        await until(lambda: bridge.status()["has_map"])
        diagnostic = DiagnosticArray(
            status=[
                DiagnosticStatus(
                    level=DiagnosticStatus.WARN, name="runtime", message="waiting for sensor"
                )
            ]
        )
        bridge._diagnostics(diagnostic)
        assert json.loads(json.dumps(bridge.status()))["diagnostics"][0]["level"] == 1
        assert bridge.grid()[0]["width"] == 4
        assert bridge.map_png().startswith(b"\x89PNG")
        await navigator.start([dict(x=1.0, y=0.0, yaw=0.0), dict(x=2.0, y=0.0, yaw=0.5)])
        await until(lambda: navigator.state == "completed")
        assert robot.goals == [1.0, 2.0]
        assert any(abs(entry[1] - 0.15) < 0.001 for entry in robot.observed)
        assert gate.output() == (0, 0, 0)
        await navigator.start([dict(x=77.0, y=0.0, yaw=0.0)])
        await until(lambda: navigator.pending is not None)
        await navigator.cancel()
        assert robot.canceled == 1
        assert gate.output() == (0, 0, 0)
        await navigator.start([dict(x=99.0, y=0.0, yaw=0.0)])
        await until(lambda: navigator.handle is not None)
        await navigator.pause()
        assert navigator.state == "paused" and robot.canceled == 2
        assert gate.output() == (0, 0, 0)
        await navigator.resume()
        await until(lambda: navigator.handle is not None)
        robot.healthy = False
        await until(lambda: not gate.nav_ready, timeout=2)
        assert gate.output() == (0, 0, 0)
        assert gate.estop
        assert bridge.status()["motion_fault"]
        robot.healthy = True
        await until(lambda: gate.nav_ready)
        assert gate.output() == (0, 0, 0)
        gate.stop(latch=True)
        await navigator.cancel()
        assert robot.canceled == 3
        assert gate.output() == (0, 0, 0)
        observed_at = time.monotonic()
        bridge.publish_velocity((0.3, 0, 0))  # A timer result computed before stop.
        await until(lambda: any(entry[0] > observed_at for entry in robot.observed))
        assert all(entry[1:] == (0, 0, 0) for entry in robot.observed if entry[0] > observed_at)

    try:
        asyncio.run(exercise())
    finally:
        gate.stop(latch=True)
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        robot.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()


def test_tracked_web_readiness_uses_rf2o_and_live_imu():
    os.environ["ROS_DOMAIN_ID"] = "221"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init()
    gate = CommandGate(lateral=0.0, navigation_limits=(0.2, 0.0, 0.4))
    gate.mode = "remote"
    bridge = RosBridge(
        gate, feedback_topic="/chassis/odom/laser",
        imu_topic="/chassis/imu/data_raw", imu_frame_id="chassis_imu_link",
        motion_enabled=True,
    )
    robot = FixtureRobot(tracked=True)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(bridge)
    executor.add_node(robot)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    async def exercise():
        await until(lambda: gate.manual_ready)
        assert bridge.status()["feedback"] and bridge.status()["imu_ready"]
        robot.healthy = False
        await until(lambda: not gate.manual_ready, timeout=2)
        assert not bridge.status()["feedback"]

    try:
        asyncio.run(exercise())
    finally:
        gate.stop(latch=True)
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        robot.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()
