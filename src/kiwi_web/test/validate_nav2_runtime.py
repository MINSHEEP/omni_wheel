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
from tf2_ros import StaticTransformBroadcaster
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from launch import LaunchDescription, LaunchService, LaunchContext
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from kiwi_web.maps import MapStore
from kiwi_web.ros_bridge import ros_future


class Sensors(Node):
    def __init__(self, pipeline="legacy"):
        super().__init__("web_nav_runtime_fixture")
        # 夹具没有运动，固定 TF 使用 transient-local 静态广播。动态广播
        # 与同时间戳 scan 的异步到达会制造不必要的 waitForTransform 请求。
        self.tf = StaticTransformBroadcaster(self)
        self.scan = self.create_publisher(
            LaserScan, "/scan_mid360" if pipeline == "mid360s" else "/scan", 1
        )
        self.odom = self.create_publisher(
            Odometry, "/fastlio/odom_base" if pipeline == "mid360s" else "/odometry/filtered", 1
        )
        transforms = []
        for parent, child, x in [
            ("map", "odom", 0.0),
            ("odom", "base_footprint", 3.0),
            ("base_footprint", "lidar_link", 0.0),
        ]:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            t.transform.translation.x = x
            transforms.append(t)
        self.tf.sendTransform(transforms)
        self.timer = self.create_timer(0.05, self.tick)

    def tick(self):
        stamp = self.get_clock().now().to_msg()
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
    pipeline = os.environ.get("KIWI_NAV_PIPELINE", "legacy")
    if pipeline not in ("legacy", "mid360s"):
        raise ValueError("KIWI_NAV_PIPELINE 必须是 legacy 或 mid360s")
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
        selector = Path(directory) / "active_platform.yaml"
        chassis = "tracked_pwm" if pipeline == "mid360s" else "kiwi"
        selector.write_text(f"chassis: {chassis}\n")
        context.launch_configurations.update(
            mode="navigation",
            map=str(store.path("room")),
            platform_file=str(selector),
            expected_chassis=chassis,
            sensor_pipeline=pipeline,
            base_params_file="",
            lidar_params_file="",
            nav2_params_file="",
        )
        actions = module.setup(context)[1:]  # Do not start real hardware.
        process_results = []

        def record_exit(event, _context):
            process_results.append((event.process_name, event.returncode))

        service = LaunchService()
        service.include_launch_description(
            LaunchDescription([RegisterEventHandler(OnProcessExit(on_exit=record_exit)), *actions])
        )
        launch_task = asyncio.create_task(service.run_async())
        rclpy.init()
        node = Sensors(pipeline)
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
            try:
                # 先停止夹具数据并让已排队的 scan/TF 回调排空，避免在 costmap
                # 销毁消息过滤器期间仍向它投递数据；executor 继续处理退出服务。
                node.timer.cancel()
                await asyncio.sleep(1)
                # 在 ROS executor 仍有效时清理 costmap 的消息过滤器与 TF 线程。
                # 同时 SIGINT 全部 active 节点会使 Humble 的清理回调互相等待。
                for manager in ("lifecycle_manager_navigation", "lifecycle_manager_localization"):
                    client = node.create_client(ManageLifecycleNodes, f"/{manager}/manage_nodes")
                    try:
                        assert await asyncio.to_thread(client.wait_for_service, timeout_sec=2), manager
                        request = ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.SHUTDOWN)
                        response = await ros_future(client.call_async(request), 10)
                        assert response.success, f"{manager} 退出失败"
                    finally:
                        node.destroy_client(client)
                print("NAV2_SHUTDOWN_PASS: lifecycle managers cleaned up before process shutdown", flush=True)
            finally:
                await service.shutdown()
                await launch_task
                executor.shutdown()
                thread.join(3)
                node.destroy_node()
                rclpy.shutdown()
                # navigation_core 展开为 8 个生命周期节点与 2 个 manager。
                assert len(process_results) == len(names) + 2, process_results
                assert all(code == 0 for _, code in process_results), process_results
                print("NAV2_EXIT_PASS: all launch processes exited with code 0", flush=True)


if __name__ == "__main__":
    asyncio.run(main())
