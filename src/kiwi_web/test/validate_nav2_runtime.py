#!/usr/bin/env python3
"""Activate the real Nav2 stack in domain 218 using test TF/scan, without hardware."""

import asyncio
import importlib.util
import os
from pathlib import Path
import tempfile
import threading
import time

os.environ["ROS_DOMAIN_ID"] = "218"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from lifecycle_msgs.srv import GetState
from launch import LaunchDescription, LaunchService, LaunchContext
from kiwi_web.maps import MapStore
from kiwi_web.ros_bridge import ros_future


class Sensors(Node):
    def __init__(self):
        super().__init__("web_nav_runtime_fixture")
        self.tf = TransformBroadcaster(self)
        self.scan = self.create_publisher(LaserScan, "/scan", 1)
        self.odom = self.create_publisher(Odometry, "/odometry/filtered", 1)
        self.create_timer(0.05, self.tick)

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for parent, child, x in [
            ("map", "odom", 0.0),
            ("odom", "base_footprint", 3.0),
            ("base_footprint", "lidar_link", 0.0),
        ]:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            t.transform.translation.x = x
            transforms.append(t)
        self.tf.sendTransform(transforms)
        s = LaserScan()
        s.header.stamp = stamp
        s.header.frame_id = "lidar_link"
        s.angle_min = -1.57
        s.angle_max = 1.57
        s.angle_increment = 0.01
        s.range_min = 0.05
        s.range_max = 20.0
        s.ranges = [5.0] * 315
        self.scan.publish(s)
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = "odom"
        o.child_frame_id = "base_footprint"
        o.pose.pose.orientation.w = 1.0
        o.pose.pose.position.x = 3.0
        self.odom.publish(o)


async def main():
    with tempfile.TemporaryDirectory(prefix="kiwi-nav-runtime-") as directory:
        store = MapStore(directory)
        store.save_grid(
            "room", dict(width=200, height=200, resolution=0.1, origin=[-5, -5, 0]), [0] * 40000
        )
        spec = importlib.util.spec_from_file_location(
            "web_stack", str(Path(__file__).resolve().parents[1] / "launch/stack.launch.py")
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        context.launch_configurations.update(
            mode="navigation",
            map=str(store.path("room")),
            base_params_file="",
            lidar_params_file="",
            nav2_params_file="",
        )
        actions = module.setup(context)[1:]  # Do not start real hardware.
        service = LaunchService()
        service.include_launch_description(LaunchDescription(actions))
        launch_task = asyncio.create_task(service.run_async())
        rclpy.init()
        node = Sensors()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        names = [
            "map_server",
            "amcl",
            "controller_server",
            "smoother_server",
            "planner_server",
            "behavior_server",
            "bt_navigator",
            "velocity_smoother",
        ]
        clients = {name: node.create_client(GetState, "/" + name + "/get_state") for name in names}
        try:
            deadline = time.monotonic() + 30
            states = {}
            while time.monotonic() < deadline:
                await asyncio.sleep(0.5)
                for name, client in clients.items():
                    if client.service_is_ready():
                        try:
                            states[name] = (
                                await ros_future(client.call_async(GetState.Request()), 2)
                            ).current_state.id
                        except asyncio.TimeoutError:
                            pass
                if len(states) == len(names) and all(v == 3 for v in states.values()):
                    break
            print("LIFECYCLE_RESULT", states, flush=True)
            assert len(states) == len(names) and all(v == 3 for v in states.values()), states
            output = node.get_publishers_info_by_topic("/cmd_vel")
            assert not output, [(e.node_name, e.topic_type) for e in output]
            gated = node.get_publishers_info_by_topic("/kiwi_web/nav_cmd_vel")
            assert any(e.node_name == "velocity_smoother" for e in gated)
            raw = node.get_publishers_info_by_topic("/kiwi_web/nav_raw")
            assert {e.node_name for e in raw} >= {"controller_server", "behavior_server"}
            print(
                "NAV2_RUNTIME_PASS: 8 lifecycle nodes active, all velocities behind web arbitration",
                flush=True,
            )
        finally:
            await service.shutdown()
            await launch_task
            executor.shutdown()
            thread.join(3)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
