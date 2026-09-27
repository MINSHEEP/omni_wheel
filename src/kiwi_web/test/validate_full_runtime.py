#!/usr/bin/env python3
"""Real web -> SLAM/Nav2 -> base serial loop, with only hardware emulated.

Localhost domain 220; private map/config/log directory. Never opens physical UARTs.
Runs production Supervisor, base/EKF/URDF, SLAM gate/toolbox, monitor and Nav2.
"""

import asyncio
import importlib.util
import math
import os
from pathlib import Path
import pty
import tempfile
import threading
import time

os.environ["ROS_DOMAIN_ID"] = "220"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters
from nav2_msgs.srv import LoadMap
from aiohttp import ClientSession, web
from ament_index_python.packages import get_package_share_directory
import yaml
from kiwi_web.control import CommandGate
from kiwi_web.maps import MapStore
from kiwi_web.ros_bridge import RosBridge, ros_future
from kiwi_web.server import Console
from kiwi_web.supervisor import Supervisor

PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "serial_protocol", PACKAGE.parent / "kiwi_base/test/test_cmd_vel_watchdog.py"
)
protocol = importlib.util.module_from_spec(spec)
spec.loader.exec_module(protocol)


class Hardware(Node):
    def __init__(self, root):
        super().__init__("web_hardware_emulator")
        self.root = root
        self.mutex = threading.Lock()
        self.fd = -1
        self.buffer = b""
        self.wheels = (0, 0, 0)
        self.frames = []
        self.x = self.y = self.yaw = 0.0
        self.feedback_enabled = True
        self.scan_enabled = True
        self.last_tick = time.monotonic()
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 1)
        self.nonzero = {
            topic: 0 for topic in ("/cmd_vel", "/kiwi_web/nav_raw", "/kiwi_web/nav_cmd_vel")
        }
        for topic in self.nonzero:
            self.create_subscription(Twist, topic, lambda m, t=topic: self.velocity(t, m), 10)
        self.create_timer(0.02, self.tick)
        self.create_timer(0.1, self.scan)
        self.params = yaml.safe_load(
            (Path(get_package_share_directory("kiwi_base")) / "config/kiwi_base.yaml").read_text()
        )
        self.base_file = root / "base.yaml"

    def velocity(self, topic, message):
        if abs(message.linear.x) + abs(message.linear.y) + abs(message.angular.z) > 0.001:
            self.nonzero[topic] += 1

    def prepare(self):
        with self.mutex:
            if self.fd >= 0:
                os.close(self.fd)
            self.fd, slave = pty.openpty()
            self.params["kiwi_base_node"]["ros__parameters"]["device"] = os.ttyname(slave)
            os.close(slave)
            os.set_blocking(self.fd, False)
            self.base_file.write_text(yaml.safe_dump(self.params))
            self.buffer = b""
            self.wheels = (0, 0, 0)
            self.x = self.y = self.yaw = 0.0
            self.last_tick = time.monotonic()

    @staticmethod
    def body_velocity(wheels):
        a, b, c = wheels
        return (
            0.107 * (a - c) / (2 * math.sin(math.pi / 3)),
            0.107 * (a + c - 2 * b) / 3,
            -0.107 * (a + b + c) / (3 * 0.35),
        )

    def tick(self):
        with self.mutex:
            if self.fd < 0:
                return
            now = time.monotonic()
            dt = min(now - self.last_tick, 0.1)
            self.last_tick = now
            frames, self.buffer = protocol.read_command_frames(self.fd, self.buffer)
            for frame in frames:
                self.frames.append((now, frame))
                self.wheels = frame
            self.frames = self.frames[-500:]
            vx, vy, wz = self.body_velocity(self.wheels)
            self.x += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
            self.y += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
            self.yaw = math.remainder(self.yaw + wz * dt, 2 * math.pi)
            # Invert the real board's configured feedback calibration.
            values = [w / 0.93457944 for w in self.wheels] + [
                0,
                0,
                self.yaw,
                0,
                0,
                -wz / 1.69,
                0,
                0,
                9.80665,
            ]
            if self.feedback_enabled:
                try:
                    os.write(self.fd, protocol.encode_feedback(values))
                except (OSError, BlockingIOError):
                    pass

    def scan(self):
        if not self.scan_enabled:
            return
        s = LaserScan()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = "lidar_link"
        s.angle_min, s.angle_increment = -math.pi, math.pi / 360
        s.angle_max = s.angle_min + 719 * s.angle_increment
        s.range_min, s.range_max, s.scan_time = 0.15, 12.0, 0.1
        # Original URDF's lidar mounting offset. The asymmetric room removes
        # square-room rotational ambiguity without publishing map->odom ourselves.
        px = self.x + 0.209 * math.cos(self.yaw)
        py = self.y + 0.209 * math.sin(self.yaw)
        ranges = []
        for index in range(720):
            angle = self.yaw + s.angle_min + index * s.angle_increment
            dx, dy = math.cos(angle), math.sin(angle)
            distances = [(edge - px) / dx for edge in (-3.0, 4.0) if abs(dx) > 1e-8]
            distances += [(edge - py) / dy for edge in (-2.5, 3.0) if abs(dy) > 1e-8]
            ranges.append(min(v for v in distances if v > 0))
        s.ranges = ranges
        self.scan_pub.publish(s)


class TestSupervisor(Supervisor):
    def __init__(self, config, hardware, root):
        super().__init__(config)
        self.hardware, self.root = hardware, root

    async def start(self, mode, map_name="", map_path=""):
        if mode != "idle":
            self.hardware.prepare()
        await super().start(mode, map_name, map_path)

    async def _read(self, process):
        with (self.root / (self.mode + ".log")).open("a") as output:
            while True:
                line = await process.stdout.readline()
                if not line:
                    break
                line = line.decode(errors="replace")
                output.write(line)
                output.flush()
                self.log.append(line.strip()[-1000:])


async def until(predicate, label, timeout=40):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            print("PASS", label, flush=True)
            return
        await asyncio.sleep(0.05)
    raise AssertionError("Timed out: " + label)


async def main():
    # Keep logs after failure for diagnosis; no production maps are touched.
    root = Path(tempfile.mkdtemp(prefix="kiwi-full-runtime-"))
    os.environ["ROS_LOG_DIR"] = str(root / "ros_logs")
    print("ARTIFACTS", root, flush=True)
    lidar = yaml.safe_load(
        (Path(get_package_share_directory("kiwi_slam")) / "config/ls_m10p_uart.yaml").read_text()
    )
    for block in lidar.values():
        block["ros__parameters"]["serial_port"] = str(root / "NO_PHYSICAL_LIDAR")
    (root / "lidar.yaml").write_text(yaml.safe_dump(lidar))
    (root / "active_platform.yaml").write_text("chassis: kiwi\n")
    rclpy.init()
    gate = CommandGate(navigation_limits=(0.8, 0.8, 1.5))
    bridge, hardware = RosBridge(gate), Hardware(root)
    supervisor = TestSupervisor(
        dict(base_params_file=str(hardware.base_file), lidar_params_file=str(root / "lidar.yaml"),
             platform_file=str(root / "active_platform.yaml"), sensor_pipeline="legacy",
             active_chassis="kiwi"),
        hardware,
        root,
    )
    # Distinct values prove all optional paths reach their owning nodes.
    for key, package, filename, node, parameter, value in (
        ("slam_params_file", "kiwi_slam", "slam_toolbox.yaml", "slam_toolbox", "resolution", 0.06),
        (
            "mapping_gate_params_file",
            "kiwi_slam",
            "mapping_gate.yaml",
            "kiwi_mapping_scan_gate",
            "required_stable_scans",
            10,
        ),
        (
            "monitor_params_file",
            "kiwi_slam",
            "monitor.yaml",
            "kiwi_slam_monitor",
            "debug_log",
            False,
        ),
        (
            "nav2_params_file",
            "kiwi_navigation",
            "nav2.yaml",
            "controller_server",
            "controller_frequency",
            15.0,
        ),
    ):
        config = yaml.safe_load(
            (Path(get_package_share_directory(package)) / "config" / filename).read_text()
        )
        config[node]["ros__parameters"][parameter] = value
        path = root / filename
        path.write_text(yaml.safe_dump(config))
        supervisor.config[key] = str(path)
    store = MapStore(root / "maps")
    console = Console(dict(port=0), bridge, gate, store, supervisor, PACKAGE / "web")
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(hardware)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    runner = web.AppRunner(console.app)
    await runner.setup()
    site = web.TCPSite(runner, "127.0.0.1", 0)
    await site.start()
    url = "http://127.0.0.1:" + str(site._server.sockets[0].getsockname()[1])
    try:
        async with ClientSession(headers={"X-Kiwi-Request": "1"}) as client:

            async def command(op, **body):
                response = await client.post(
                    url + "/api/command", json=dict(op=op, mode_id=console.mode_id, **body)
                )
                result = await response.json()
                assert response.status == 200, (op, result)
                return result.get("result")

            async def parameter(node, name):
                service = bridge.create_client(GetParameters, "/" + node + "/get_parameters")
                await until(service.service_is_ready, "parameter service " + node)
                result = await ros_future(
                    service.call_async(GetParameters.Request(names=[name])), 5
                )
                bridge.destroy_client(service)
                return result.values[0]

            await command("mode", mode="mapping")
            await until(
                lambda: gate.manual_ready and bridge.map_message is not None,
                "real base calibration + EKF + scan gate + SLAM map",
                50,
            )
            await until(
                lambda: any(d["name"] == "kiwi_slam/map" for d in bridge.diagnostics),
                "original monitor diagnostics received",
            )
            assert abs((await parameter("slam_toolbox", "resolution")).double_value - 0.06) < 1e-8
            assert (
                await parameter("kiwi_mapping_scan_gate", "required_stable_scans")
            ).integer_value == 10
            assert not (await parameter("kiwi_slam_monitor", "debug_log")).bool_value
            assert (await parameter("kiwi_base_node", "device")).string_value.startswith(
                "/dev/pts/"
            )
            assert (await parameter("ekf_filter_node", "odom0")).string_value == "/chassis/odom/raw"
            print(
                "PASS base/EKF/SLAM/gate/monitor parameter overrides reach actual nodes", flush=True
            )
            status = await (await client.get(url + "/api/status")).json()
            assert status["robot"]["diagnostics"]
            assert all(type(d["level"]) is int for d in status["robot"]["diagnostics"])
            print("PASS diagnostics HTTP serialization", flush=True)
            async with client.ws_connect(url + "/ws", origin=url) as socket:

                async def receive(kind):
                    while True:
                        message = await socket.receive_json(timeout=3)
                        if message["type"] == "error":
                            raise AssertionError(message)
                        if message["type"] == kind:
                            return message

                await receive("hello")
                for velocity in ([0.15, 0, 0], [0, 0.15, 0], [0, 0, 0.3]):
                    await socket.send_json(dict(op="acquire"))
                    await receive("acquired")
                    state = await receive("state")
                    start = time.monotonic()
                    await socket.send_json(
                        dict(op="drive", velocity=velocity, seq=0, challenge=state["challenge"])
                    )
                    await until(
                        lambda: any(
                            t > start
                            and max(abs(a - b) for a, b in zip(hardware.body_velocity(w), velocity))
                            < 0.001
                            for t, w in hardware.frames
                        ),
                        "WS -> /cmd_vel -> real C++ -> serial axis " + str(velocity),
                        2,
                    )
                    await until(
                        lambda: any(
                            t > start + 0.24 and max(map(abs, w)) < 0.0001
                            for t, w in hardware.frames
                        ),
                        "250 ms web watchdog reaches serial stop",
                        1,
                    )
                    await socket.send_json(dict(op="release"))
                state = await receive("state")
                assert state["robot"]["diagnostics"]
                print("PASS diagnostics WebSocket serialization", flush=True)
            await asyncio.sleep(2)
            original = await command("save_map", name="web_room")
            assert (store.path("web_room")).is_file()
            print("PASS web saves real SLAM PGM/YAML", original, flush=True)
            await command("mode", mode="idle")
            doc = store.read("web_room")
            # Edit pixels away from the route: obstacle, unknown, then free.
            image = store.occupancy_image(doc)
            width, height = image.size
            indices = [width * 10 + 10, width * 10 + 11, width * 10 + 12]
            await command(
                "edit_map",
                name="web_room",
                revision=doc["revision"],
                operations=[dict(value=v, indices=[i]) for v, i in zip((0, 205, 254), indices)],
            )
            edited = store.occupancy_image(store.read("web_room")).tobytes()
            await command("mode", mode="navigation", map="web_room")
            await until(
                lambda: bridge.map_message is not None, "real map_server loads edited YAML/PGM"
            )
            grid = bridge.map_message
            for i, expected in zip(indices, (100, -1, 0)):
                ros_index = (height - 1 - i // width) * width + i % width
                assert grid.data[ros_index] == expected, (i, grid.data[ros_index], expected)
            # Compare every pixel after ROS row flip, not just our edited cells.
            expected_grid = [
                (-1 if p == 205 else 100 if p == 0 else 0)
                for y in range(height - 1, -1, -1)
                for p in edited[y * width : (y + 1) * width]
            ]
            assert list(grid.data) == expected_grid
            print("PASS every edited map pixel survives real map_server", flush=True)
            await until(
                lambda: bridge.count_subscribers("/initialpose") > 0 and bridge.imu_ready,
                "AMCL initial pose subscriber + real base calibration",
                50,
            )
            await command("initial_pose", pose=dict(x=0.0, y=0.0, yaw=0.0))
            await until(
                lambda: bridge.navigation.server_is_ready() and gate.nav_ready,
                "web initial pose -> real AMCL map/odom TF + Nav2 action",
                30,
            )
            assert bridge.pose["frame"] == "map"
            assert len(bridge.scan) > 500
            assert (
                await parameter("controller_server", "controller_frequency")
            ).double_value == 15.0
            assert (await parameter("kiwi_slam_monitor", "map_is_static")).bool_value
            print("POSE", bridge.pose, "LASER_POINTS", len(bridge.scan), flush=True)
            await command(
                "navigate", points=[dict(x=1.0, y=0.0, yaw=0.0), dict(x=1.4, y=0.0, yaw=0.0)]
            )
            await until(
                lambda: all(v > 0 for v in hardware.nonzero.values()),
                "planner/controller -> smoother -> web gate -> real base serial",
                25,
            )
            await command("pause")
            assert console.navigator.state == "paused"
            await until(
                lambda: max(map(abs, hardware.wheels)) < 0.0001,
                "real Nav2 pause reaches serial zero",
                2,
            )
            await command("resume")
            await until(
                lambda: console.navigator.state in ("completed", "failed"),
                "real NavigateToPose result",
                65,
            )
            assert console.navigator.state == "completed", (
                console.navigator.status(),
                bridge.status(),
                list(supervisor.log),
            )
            assert console.navigator.index == 2
            assert abs(hardware.x - 1.4) < 0.35 and abs(hardware.y) < 0.35, (hardware.x, hardware.y)
            print(
                "PASS closed-loop Nav2 succeeded",
                hardware.x,
                hardware.y,
                "COUNTS",
                hardware.nonzero,
                flush=True,
            )
            await command("navigate", points=[dict(x=2.0, y=0.0, yaw=0.0)])
            await until(
                lambda: gate.nav_enabled and any(abs(v) > 0.02 for v in gate.output()),
                "second navigation moving",
            )
            hardware.scan_enabled = False
            await until(
                lambda: gate.estop and not any(gate.output()), "laser loss latches web stop", 2
            )
            await until(
                lambda: max(map(abs, hardware.wheels)) < 0.0001, "laser loss reaches serial zero", 1
            )
            hardware.scan_enabled = True
            await until(lambda: gate.nav_ready, "sensor restored")
            await asyncio.sleep(0.4)
            assert gate.estop and not any(gate.output())
            await command("cancel")
            # Original maps are read without changing a byte. LoadMap uses the
            # real map_server on this isolated graph with motion still latched.
            loader = bridge.create_client(LoadMap, "/map_server/load_map")
            existing = MapStore(PACKAGE.parents[1] / "maps")
            for metadata in existing.list():
                name = metadata["name"]
                document = existing.read(name)
                result = await ros_future(
                    loader.call_async(LoadMap.Request(map_url=str(existing.path(name)))), 10
                )
                assert result.result == 0, name
                pixels = existing.occupancy_image(document)
                w, h = pixels.size
                data = pixels.tobytes()
                expected = [
                    (-1 if p == 205 else 100 if p == 0 else 0)
                    for y in range(h - 1, -1, -1)
                    for p in data[y * w : (y + 1) * w]
                ]
                assert list(result.map.data) == expected, name
                print("PASS original workspace map_server pixels", name, w, h, flush=True)
            bridge.destroy_client(loader)
            await command("mode", mode="idle")
            print("FULL_RUNTIME_PASS", root, flush=True)
    except Exception:
        print("FAILED_STATE", console.snapshot(), flush=True)
        raise
    finally:
        await runner.cleanup()
        executor.shutdown(timeout_sec=3)
        thread.join(3)
        if hardware.fd >= 0:
            os.close(hardware.fd)
        hardware.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
