"""ROS subscriptions and actions; transport stays off the 50 Hz control loop."""

import asyncio
from collections import deque
import io
import math
import threading
import time

from PIL import Image, ImageOps
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformException, TransformListener

from .maps import MAX_CELLS


def yaw(quaternion):
    return math.atan2(
        2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1 - 2 * (quaternion.y**2 + quaternion.z**2),
    )


async def ros_future(future, timeout=10):
    loop = asyncio.get_running_loop()
    result = loop.create_future()

    def resolve(completed):
        if result.done():
            return
        try:
            result.set_result(completed.result())
        except Exception as error:
            result.set_exception(error)

    future.add_done_callback(lambda completed: loop.call_soon_threadsafe(resolve, completed))
    return await asyncio.wait_for(result, timeout)


class RosBridge(Node):
    def __init__(self, gate):
        super().__init__("kiwi_web_bridge")
        self.gate = gate
        self.lock = threading.RLock()
        self.encoding_lock = threading.Lock()
        self.map_message = None
        self.map_version = 0
        self.map_cache = None
        self.odom_at = self.scan_at = self.feedback_at = 0.0
        self.imu_ready = self.mapping_ready = False
        self.pose = None
        self.scan = []
        self.pending_scans = deque(maxlen=5)
        self.path = []
        self.speed = [0.0, 0.0, 0.0]
        self.diagnostics = []
        self.diagnostic_entries = {}
        self.conflicts = []
        self.motion_fault = ""
        self.pose_at = 0.0
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.command_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self.create_subscription(OccupancyGrid, "/map", self._map, latched)
        self.create_subscription(
            Odometry, "/odometry/filtered", self._odom, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/chassis/odom/raw", self._feedback, qos_profile_sensor_data
        )
        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.create_subscription(Path, "/plan", self._path, 1)
        self.create_subscription(Bool, "/chassis/imu/calibration_ready", self._imu, latched)
        self.create_subscription(
            Bool, "/kiwi_slam/mapping_gate_ready", self._mapping_ready, latched
        )
        self.create_subscription(DiagnosticArray, "/diagnostics", self._diagnostics, 5)
        self.create_subscription(DiagnosticArray, "/kiwi_slam/diagnostics", self._diagnostics, 5)
        self.create_subscription(Twist, "/kiwi_web/nav_cmd_vel", self._nav_velocity, 1)
        self.navigation = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.navigation_state = self.create_client(GetState, "/bt_navigator/get_state")
        self.navigation_active = False
        self.state_future = None
        self.graph_epoch = 0
        self.clear_clients = [
            self.create_client(ClearEntireCostmap, topic)
            for topic in (
                "/global_costmap/clear_entirely_global_costmap",
                "/local_costmap/clear_entirely_local_costmap",
            )
        ]
        self.create_timer(0.02, self._control)
        self.create_timer(0.1, self._transform)
        self.create_timer(1.0, self._check_publishers)

    def _fresh_stamp(self, stamp, maximum=0.6):
        age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
        return -0.1 <= age <= maximum

    def _map(self, message):
        if (
            message.header.frame_id != "map"
            or not 0 < message.info.width * message.info.height <= MAX_CELLS
            or len(message.data) != message.info.width * message.info.height
            or not math.isfinite(message.info.resolution)
            or message.info.resolution <= 0
        ):
            return
        with self.lock:
            self.map_message = message
            self.map_version += 1

    def _odom(self, message):
        if not self._fresh_stamp(message.header.stamp):
            return
        with self.lock:
            self.odom_at = time.monotonic()
            self.speed = [
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.angular.z,
            ]

    def _feedback(self, message):
        if self._fresh_stamp(message.header.stamp):
            with self.lock:
                self.feedback_at = time.monotonic()

    def _imu(self, message):
        with self.lock:
            self.imu_ready = message.data

    def _mapping_ready(self, message):
        with self.lock:
            self.mapping_ready = message.data

    def _diagnostics(self, message):
        with self.lock:
            now = time.monotonic()
            for s in message.status:
                self.diagnostic_entries[s.name] = (
                    now,
                    dict(
                        name=s.name,
                        level=(
                            int.from_bytes(s.level, "little")
                            if isinstance(s.level, bytes)
                            else int(s.level)
                        ),
                        message=s.message,
                    ),
                )
            self.diagnostic_entries = {
                key: value for key, value in self.diagnostic_entries.items() if now - value[0] < 3.0
            }
            self.diagnostics = [value[1] for value in self.diagnostic_entries.values()][:30]

    def _scan(self, message):
        if not self._fresh_stamp(message.header.stamp):
            return
        with self.lock:
            self.scan_at = time.monotonic()
            self.pending_scans.append(message)
        self._update_scan()

    def _update_scan(self):
        # Scan normally arrives before the EKF transform for its timestamp.
        # Retry a bounded queue from the timer instead of discarding that scan.
        with self.lock:
            pending = list(self.pending_scans)
        frame = "map" if self.gate.mode in ("mapping", "navigation") else "odom"
        message = None
        for candidate in reversed(pending):
            if not self._fresh_stamp(candidate.header.stamp, maximum=0.5):
                continue
            try:
                transform = self.tf.lookup_transform(
                    frame, candidate.header.frame_id, Time.from_msg(candidate.header.stamp)
                )
                message = candidate
                break
            except TransformException:
                continue
        if message is None:
            return
        angle = yaw(transform.transform.rotation)
        tx, ty = transform.transform.translation.x, transform.transform.translation.y
        step = max(1, math.ceil(len(message.ranges) / 720))
        points = []
        for index in range(0, len(message.ranges), step):
            distance = message.ranges[index]
            if math.isfinite(distance) and message.range_min <= distance <= message.range_max:
                theta = message.angle_min + index * message.angle_increment + angle
                points.append(
                    [
                        round(tx + distance * math.cos(theta), 3),
                        round(ty + distance * math.sin(theta), 3),
                    ]
                )
        with self.lock:
            self.scan = points
            while self.pending_scans:
                if self.pending_scans.popleft() is message:
                    break

    def _path(self, message):
        if message.header.frame_id == "map":
            step = max(1, math.ceil(len(message.poses) / 1500))
            with self.lock:
                self.path = [
                    dict(x=p.pose.position.x, y=p.pose.position.y) for p in message.poses[::step]
                ]

    def _nav_velocity(self, message):
        try:
            self.gate.navigation([message.linear.x, message.linear.y, message.angular.z])
        except ValueError:
            self.gate.navigation([0.0, 0.0, 0.0])

    def _transform(self):
        self._update_scan()
        frame = "map" if self.gate.mode in ("mapping", "navigation") else "odom"
        try:
            transform = self.tf.lookup_transform(frame, "base_footprint", Time())
            if not self._fresh_stamp(transform.header.stamp):
                raise ValueError("stale transform")
            with self.lock:
                self.pose = dict(
                    x=transform.transform.translation.x,
                    y=transform.transform.translation.y,
                    yaw=yaw(transform.transform.rotation),
                    frame=frame,
                )
                self.pose_at = time.monotonic()
        except (TransformException, ValueError):
            with self.lock:
                self.pose = None

    def _check_publishers(self):
        self._check_navigation_state()
        entries = self.get_publishers_info_by_topic("/cmd_vel")
        with self.lock:
            self.conflicts = [
                e.node_namespace + "/" + e.node_name
                for e in entries
                if e.node_name != self.get_name() or e.node_namespace != self.get_namespace()
            ]
            # A second instance with the same node name is a conflict too.
            if len(entries) > 1 and not self.conflicts:
                self.conflicts = ["重复的 kiwi_web_bridge"]

    def _check_navigation_state(self):
        if self.gate.mode != "navigation" or not self.navigation_state.service_is_ready():
            self.navigation_active = False
            return
        if self.state_future is not None and not self.state_future.done():
            return
        epoch = self.graph_epoch
        self.state_future = self.navigation_state.call_async(GetState.Request())

        def completed(future):
            if epoch != self.graph_epoch:
                return
            try:
                self.navigation_active = future.result().current_state.id == 3
            except Exception:
                self.navigation_active = False

        self.state_future.add_done_callback(completed)

    def _control(self):
        now = time.monotonic()
        with self.lock:
            healthy = (
                now - self.feedback_at < 0.5 and now - self.odom_at < 0.5 and not self.conflicts
            )
            manual_ready = (
                healthy
                and self.imu_ready
                and (
                    self.gate.mode == "remote"
                    or (self.gate.mode == "mapping" and self.mapping_ready)
                )
            )
            nav_ready = (
                healthy
                and self.navigation_active
                and self.imu_ready
                and now - self.scan_at < 0.5
                and self.pose is not None
                and now - self.pose_at < 0.5
            )
        with self.gate.lock:
            self.gate.manual_ready = manual_ready
            self.gate.nav_ready = nav_ready
            if not manual_ready and self.gate.owner:
                self.gate.release()
            if self.gate.nav_enabled and not nav_ready:
                self.gate.stop(latch=True)
                with self.lock:
                    self.motion_fault = (
                        "导航期间底盘、雷达或定位反馈中断，已锁定停车；请检查设备后解除软件急停"
                    )
        self.publish_velocity(self.gate.output())

    def publish_velocity(self, velocity):
        # The ROS timer and HTTP thread can race with stop/release. Re-evaluate
        # a nonzero command while holding the same lock as the stop operation.
        with self.gate.lock:
            if any(velocity):
                velocity = self.gate.output()
            message = Twist()
            message.linear.x, message.linear.y, message.angular.z = map(float, velocity)
            self.command_pub.publish(message)

    def reset_data(self):
        with self.lock:
            self.graph_epoch += 1
            self.navigation_active = False
            self.state_future = None
            self.map_message = self.map_cache = None
            self.map_version += 1
            self.path, self.scan = [], []
            self.diagnostics = []
            self.diagnostic_entries.clear()
            self.pending_scans.clear()
            self.pose = None
            self.imu_ready = self.mapping_ready = False
            self.odom_at = self.scan_at = self.feedback_at = self.pose_at = 0.0
            self.tf.clear()

    def clear_motion_fault(self):
        with self.lock:
            self.motion_fault = ""

    def status(self):
        now = time.monotonic()
        with self.lock:
            return dict(
                motion_fault=self.motion_fault,
                odom=now - self.odom_at < 0.5,
                feedback=now - self.feedback_at < 0.5,
                lidar=now - self.scan_at < 0.5,
                imu_ready=self.imu_ready,
                mapping_ready=self.mapping_ready,
                pose=self.pose,
                speed=self.speed,
                scan=self.scan if now - self.scan_at < 0.5 else [],
                path=self.path,
                map_version=self.map_version,
                has_map=self.map_message is not None,
                diagnostics=self.diagnostics,
                conflicts=self.conflicts,
                action_ready=self.navigation_active and self.navigation.server_is_ready(),
            )

    def grid(self):
        with self.lock:
            message = self.map_message
            version = self.map_version
        if message is None:
            raise ValueError("尚未收到地图，请等待建图门控就绪")
        info = message.info
        return (
            dict(
                width=info.width,
                height=info.height,
                resolution=info.resolution,
                origin=[
                    info.origin.position.x,
                    info.origin.position.y,
                    yaw(info.origin.orientation),
                ],
                version=version,
            ),
            message.data,
        )

    def map_png(self):
        # Multiple viewers share one encoding per map version. Pillow applies
        # the palette in native code instead of iterating a large grid in Python.
        with self.encoding_lock:
            return self._encode_map_png()

    def _encode_map_png(self):
        info, data = self.grid()
        with self.lock:
            if self.map_cache and self.map_cache[0] == info["version"]:
                return self.map_cache[1]
        raw = data.tobytes()
        palette = [
            round(254 * (100 - value) / 100) if value <= 100 else 205 for value in range(256)
        ]
        image = ImageOps.flip(
            Image.frombytes("L", (info["width"], info["height"]), raw).point(palette)
        )
        buffer = io.BytesIO()
        image.save(buffer, "PNG", compress_level=2)
        with self.lock:
            self.map_cache = (info["version"], buffer.getvalue())
        return buffer.getvalue()

    def pose_message(self, point):
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x, message.pose.position.y = point["x"], point["y"]
        message.pose.orientation.z = math.sin(point["yaw"] / 2)
        message.pose.orientation.w = math.cos(point["yaw"] / 2)
        return message

    def initial_pose(self, point):
        if not self.count_subscribers("/initialpose"):
            raise ValueError("AMCL 尚未就绪，请稍后重试")
        stamped = self.pose_message(point)
        message = PoseWithCovarianceStamped(header=stamped.header)
        message.pose.pose = stamped.pose
        message.pose.covariance[0] = message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.06854
        self.initial_pub.publish(message)

    async def clear_costmaps(self):
        if not all(client.service_is_ready() for client in self.clear_clients):
            raise ValueError("导航代价地图服务尚未就绪")
        await asyncio.gather(
            *(
                ros_future(client.call_async(ClearEntireCostmap.Request()))
                for client in self.clear_clients
            )
        )


class Navigator:
    """A route is sequential NavigateToPose actions; pause cancels the active goal."""

    def __init__(self, bridge, gate):
        self.bridge, self.gate = bridge, gate
        self.points = []
        self.index = 0
        self.state = "idle"
        self.message = ""
        self.feedback = {}
        self.handle = None
        self.pending = None
        self.cancel_requests = {}
        self.task = None
        self.epoch = 0

    def status(self):
        return dict(
            state=self.state,
            message=self.message,
            index=self.index,
            total=len(self.points),
            points=self.points,
            **self.feedback
        )

    async def start(self, points):
        if self.gate.estop or self.gate.mode != "navigation" or not self.gate.nav_ready:
            raise ValueError("请先加载地图、完成定位并确认底盘与雷达在线")
        if not self.bridge.navigation.server_is_ready():
            raise ValueError("Nav2 尚未激活，请稍后重试")
        await self.cancel()
        self.points, self.index = points, 0
        self._run()

    def _run(self):
        self.state, self.message = "running", ""
        self.epoch += 1
        self.task = asyncio.create_task(self._execute(self.epoch))

    async def _execute(self, epoch):
        loop = asyncio.get_running_loop()

        def feedback(message):
            def update():
                if epoch == self.epoch:
                    item = message.feedback
                    self.feedback = dict(
                        distance=round(item.distance_remaining, 2),
                        remaining=item.estimated_time_remaining.sec,
                        recoveries=item.number_of_recoveries,
                    )

            loop.call_soon_threadsafe(update)

        try:
            while self.index < len(self.points):
                request = NavigateToPose.Goal(
                    pose=self.bridge.pose_message(self.points[self.index])
                )
                future = self.bridge.navigation.send_goal_async(request, feedback_callback=feedback)
                self.pending = future

                # If acceptance is delayed beyond cancellation/timeout, cancel it
                # on arrival; it must never become an untracked moving goal.
                def late_accept(completed):
                    def cancel_stale():
                        if epoch != self.epoch:
                            try:
                                handle = completed.result()
                            except Exception:
                                # The original waiter reports request failures;
                                # a failed request has no accepted goal to stop.
                                return
                            if handle and handle.accepted:
                                self.request_cancel(handle)

                    loop.call_soon_threadsafe(cancel_stale)

                future.add_done_callback(late_accept)
                self.handle = await ros_future(future, 10)
                self.pending = None
                if not self.handle.accepted:
                    raise ValueError("Nav2 拒绝目标点")
                self.gate.nav_enabled = True
                result = await ros_future(self.handle.get_result_async(), 3600)
                self.gate.nav_enabled = False
                self.handle = None
                if result.status != 4:
                    raise ValueError("导航未完成（状态 %s）" % result.status)
                self.index += 1
            self.state = "completed"
            self.message = "全部目标点已到达"
        except asyncio.CancelledError:
            raise
        except Exception as error:
            self.state, self.message = "failed", str(error)
            self.epoch += 1
            if self.handle and self.handle.accepted:
                self.request_cancel(self.handle)
        finally:
            self.gate.nav_enabled = False
            self.bridge.publish_velocity((0, 0, 0))

    def request_cancel(self, handle):
        key = bytes(handle.goal_id.uuid)
        if key not in self.cancel_requests:
            self.cancel_requests[key] = handle.cancel_goal_async()
            if len(self.cancel_requests) > 8:
                self.cancel_requests.pop(next(iter(self.cancel_requests)))
        return self.cancel_requests[key]

    async def cancel(self, state="canceled"):
        self.gate.stop()
        self.epoch += 1
        if self.task and not self.task.done():
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)
        if self.pending is not None:
            try:
                self.handle = await ros_future(self.pending, 5)
                self.pending = None
            except asyncio.TimeoutError:
                self.state, self.message = "failed", "目标确认超时，请停止导航模式后重启"
                self.gate.stop(latch=True)
                raise ValueError(self.message)
        if self.handle and self.handle.accepted:
            handle = self.handle
            try:
                await ros_future(self.request_cancel(handle), 5)
                await ros_future(handle.get_result_async(), 5)
            except asyncio.TimeoutError:
                self.state, self.message = "failed", "Nav2 取消超时，请停止导航模式后重启"
                self.gate.stop(latch=True)
                raise ValueError(self.message)
        self.handle = None
        self.state = state
        self.feedback = {}
        with self.bridge.lock:
            self.bridge.path = []

    async def pause(self):
        if self.state != "running":
            raise ValueError("没有运行中的路径")
        await self.cancel("paused")

    async def resume(self):
        if self.state != "paused" or self.index >= len(self.points):
            raise ValueError("没有可继续的路径")
        if self.gate.estop or not self.gate.nav_ready:
            raise ValueError("导航未就绪或软件急停未解除")
        self._run()

    async def skip(self):
        if self.state not in ("running", "paused"):
            raise ValueError("没有可跳过的目标点")
        was_running = self.state == "running"
        await self.cancel("paused")
        self.index += 1
        if self.index >= len(self.points):
            self.state = "completed"
        elif was_running:
            await self.resume()
