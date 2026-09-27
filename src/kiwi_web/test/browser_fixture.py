#!/usr/bin/env python3
"""Test-only browser fixture: localhost:8081, domain 217, no hardware processes."""

import os
from pathlib import Path
import tempfile
import threading

from aiohttp import web
import rclpy
from rclpy.executors import MultiThreadedExecutor

from kiwi_web.control import CommandGate
from kiwi_web.maps import MapStore
from kiwi_web.ros_bridge import RosBridge
from kiwi_web.server import Console
from test_ros_integration import FixtureRobot


class FixtureSupervisor:
    def __init__(self, robot, store):
        self.mode = "idle"
        self.map_name = ""
        self.process = None
        self.error = ""
        self.log = ["隔离的自动化测试环境，不连接硬件"]
        self.robot, self.store = robot, store

    async def start(self, mode, map_name="", map_path=""):
        self.mode, self.map_name = mode, map_name
        if mode == "navigation":
            document = self.store.read(map_name)
            image = self.store.occupancy_image(document)
            width, height = image.size
            pixels = image.tobytes()
            self.robot.grid_data = [
                (
                    -1
                    if pixels[(height - 1 - y) * width + x] == 205
                    else 100 if pixels[(height - 1 - y) * width + x] == 0 else 0
                )
                for y in range(height)
                for x in range(width)
            ]
            self.robot.last_map = 0.0

    async def stop(self):
        self.mode, self.map_name = "idle", ""

    def check(self):
        pass


def main():
    chassis = os.environ.get("KIWI_BROWSER_CHASSIS", "kiwi")
    if chassis not in ("kiwi", "tracked_pwm"):
        raise ValueError("KIWI_BROWSER_CHASSIS 必须是 kiwi 或 tracked_pwm")
    pipeline = os.environ.get("KIWI_BROWSER_PIPELINE", "legacy")
    if pipeline not in ("legacy", "mid360s") or (pipeline == "mid360s" and chassis != "tracked_pwm"):
        raise ValueError("KIWI_BROWSER_PIPELINE 必须是 legacy；履带底盘还可选 mid360s")
    tracked = chassis == "tracked_pwm"
    port = 8083 if tracked else 8081
    os.environ["ROS_DOMAIN_ID"] = "222" if tracked else "217"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init()
    width, height = 160, 120
    cells = []
    for y in range(height):
        for x in range(width):
            if x < 10 or y < 10 or x >= width - 10 or y >= height - 10:
                cells.append(-1)
            elif (
                x in (10, 149)
                or y in (10, 109)
                or (x in (55, 105) and y < 90)
                or (y == 60 and 55 < x < 105)
            ):
                cells.append(100)
            else:
                cells.append(0)
    gate = CommandGate(lateral=0.0 if tracked else 0.4)
    bridge = RosBridge(
        gate, feedback_topic="/chassis/odom/laser" if tracked else "/chassis/odom/raw",
        imu_topic="/chassis/imu/data_raw" if tracked else "",
        motion_enabled=True,
    )
    robot = FixtureRobot(width, height, cells, tracked=tracked)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    executor.add_node(robot)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    temporary = tempfile.TemporaryDirectory(prefix="kiwi-browser-")
    store = MapStore(temporary.name)
    store.save_grid(
        "fixture_lab", dict(width=width, height=height, resolution=0.1, origin=[0, 0, 0]), cells
    )
    console = Console(
        dict(port=port, active_chassis=chassis, sensor_pipeline=pipeline),
        bridge,
        gate,
        store,
        FixtureSupervisor(robot, store),
        Path(__file__).resolve().parents[1] / "web",
    )
    try:
        web.run_app(console.app, host="127.0.0.1", port=port, access_log=None)
    finally:
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        robot.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()
        temporary.cleanup()


if __name__ == "__main__":
    main()
