"""One-origin HTTP/WebSocket console, backed by ROS 2 and local map storage."""

import argparse
import asyncio
from contextlib import suppress
import fcntl
import hmac
import json
import logging
import os
from pathlib import Path
import secrets
import threading
import time

from aiohttp import web, WSMsgType
from ament_index_python.packages import get_package_share_directory
import psutil
import rclpy
from rclpy.executors import SingleThreadedExecutor
import yaml

from .control import CommandGate
from kiwi_platform import load_profile, validate_profile
from kiwi_platform.launching import validate_map_file
from .config import PARAMETER_FILES, resolve_paths
from .maps import MapStore, finite, pose
from .ros_bridge import Navigator, RosBridge
from .supervisor import Supervisor

LOG = logging.getLogger(__name__)


class Console:
    def __init__(self, config, bridge, gate, store, supervisor, web_root):
        self.config, self.bridge, self.gate = config, bridge, gate
        self.store, self.supervisor = store, supervisor
        self.web_root = Path(web_root)
        self.navigator = Navigator(bridge, gate)
        self.clients = {}
        self.sessions = {}
        self.login_attempts = {}
        self.operation = asyncio.Lock()
        self.transitioning = False
        self.mode_id = secrets.token_hex(8)
        self.system = {}
        self.last_system = 0.0
        self.token = os.environ.get("KIWI_WEB_TOKEN", config.get("access_token", ""))
        self.app = web.Application(client_max_size=64 * 1024 * 1024, middlewares=[self.middleware])
        self.app.add_routes(
            [
                web.get("/api/config", self.public_config),
                web.post("/api/login", self.login),
                web.get("/api/status", self.status),
                web.get("/api/maps", self.maps),
                web.get("/api/maps/{name}/image", self.map_image),
                web.get("/api/maps/{name}/export", self.map_export),
                web.get("/api/maps/{name}/routes", self.routes),
                web.get("/api/maps/{name}", self.map_metadata),
                web.get("/api/live/map", self.live_metadata),
                web.get("/api/live/image", self.live_image),
                web.post("/api/command", self.command),
                web.post("/api/import", self.import_map),
                web.get("/ws", self.websocket),
                web.get("/{path:.*}", self.static),
            ]
        )
        self.app.on_startup.append(self.startup)
        self.app.on_shutdown.append(self.shutdown)

    @web.middleware
    async def middleware(self, request, handler):
        try:
            if request.path.startswith("/api/") or request.path == "/ws":
                origin = request.headers.get("Origin")
                expected = request.scheme + "://" + request.host
                if origin and origin != expected:
                    raise web.HTTPForbidden(text="拒绝跨站请求")
                if request.method != "GET" and request.headers.get("X-Kiwi-Request") != "1":
                    raise web.HTTPForbidden(text="缺少同源请求标识")
                if request.path not in ("/api/config", "/api/login") and not self.authorized(
                    request
                ):
                    raise web.HTTPUnauthorized(text="请先输入访问密码")
            result = await handler(request)
            result.headers["X-Content-Type-Options"] = "nosniff"
            result.headers["Referrer-Policy"] = "same-origin"
            if request.path.startswith("/api/"):
                result.headers["Cache-Control"] = "no-store"
            return result
        except web.HTTPException as error:
            if request.path.startswith("/api/"):
                return web.json_response({"error": error.text}, status=error.status)
            raise
        except FileNotFoundError:
            return web.json_response({"error": "地图或文件不存在"}, status=404)
        except (ValueError, KeyError, TypeError, yaml.YAMLError) as error:
            return web.json_response({"error": str(error)}, status=400)
        except Exception:
            LOG.exception("Web request failed")
            return web.json_response({"error": "服务处理失败，请查看设备日志"}, status=500)

    def authorized(self, request):
        return (
            not self.token
            or self.sessions.get(request.cookies.get("kiwi_session"), 0) > time.monotonic()
        )

    async def public_config(self, request):
        return web.json_response(
            dict(
                auth_required=not self.authorized(request),
                chassis=self.config.get("active_chassis", "kiwi"),
                limits=self.gate.limits,
                motion_enabled=self.config.get("motion_enabled", True),
                port=self.config["port"],
            )
        )

    async def login(self, request):
        now = time.monotonic()
        attempts = [t for t in self.login_attempts.get(request.remote, []) if now - t < 60]
        if len(attempts) >= 10:
            raise web.HTTPTooManyRequests(text="请稍后再试")
        attempts.append(now)
        self.login_attempts[request.remote] = attempts
        body = await request.json()
        if not isinstance(body.get("token"), str) or not hmac.compare_digest(
            body["token"], self.token
        ):
            raise web.HTTPUnauthorized(text="访问密码不正确")
        self.sessions = {k: t for k, t in self.sessions.items() if t > now}
        token = secrets.token_urlsafe(32)
        self.sessions[token] = now + 86400
        response = web.json_response({"ok": True})
        response.set_cookie(
            "kiwi_session",
            token,
            httponly=True,
            samesite="Strict",
            secure=request.secure,
            max_age=86400,
        )
        return response

    def snapshot(self):
        return dict(
            chassis=self.config.get("active_chassis", "kiwi"),
            motion_enabled=self.config.get("motion_enabled", True),
            mode=self.supervisor.mode,
            mode_id=self.mode_id,
            current_map=self.supervisor.map_name,
            transitioning=self.transitioning,
            error=self.supervisor.error,
            control=self.gate.status(),
            robot=self.bridge.status(),
            navigation=self.navigator.status(),
            system=self.system,
            logs=list(self.supervisor.log)[-12:],
        )

    async def status(self, request):
        return web.json_response(self.snapshot())

    async def maps(self, request):
        return web.json_response(await asyncio.to_thread(self.store.list))

    async def map_metadata(self, request):
        document = await asyncio.to_thread(self.store.read, request.match_info["name"])
        return web.json_response(self.store.public(document))

    async def map_image(self, request):
        data, revision = await asyncio.to_thread(self.store.png, request.match_info["name"])
        if request.query.get("revision", revision) != revision:
            raise ValueError("地图版本已变化，请刷新")
        return web.Response(body=data, content_type="image/png", headers={"ETag": revision})

    async def map_export(self, request):
        name = request.match_info["name"]
        data = await asyncio.to_thread(self.store.export, name)
        return web.Response(
            body=data,
            content_type="application/zip",
            headers={"Content-Disposition": 'attachment; filename="' + name + '.zip"'},
        )

    async def routes(self, request):
        return web.json_response(
            await asyncio.to_thread(self.store.list_routes, request.match_info["name"])
        )

    async def live_metadata(self, request):
        info, _ = self.bridge.grid()
        return web.json_response(info)

    async def live_image(self, request):
        version = self.bridge.grid()[0]["version"]
        data = await asyncio.to_thread(self.bridge.map_png)
        if (
            request.query.get("version", str(version)) != str(version)
            or self.bridge.grid()[0]["version"] != version
        ):
            raise ValueError("地图已刷新，请重试")
        return web.Response(body=data, content_type="image/png")

    async def change_mode(self, mode, name=""):
        if mode not in ("idle", "mapping", "navigation", "remote"):
            raise ValueError("不支持的模式")
        if mode == self.supervisor.mode and name == self.supervisor.map_name:
            return
        map_path = ""
        if mode == "navigation":
            await asyncio.to_thread(self.store.read, name)
            map_path = self.store.path(name)
            # Reject an invalid navigation configuration before stopping mapping.
            # Production load_config always resolves platform_file; test fixtures
            # without a physical platform keep their synthetic ROS graph.
            if self.config.get("platform_file"):
                overrides = {
                    key: self.config.get(argument, "")
                    for argument, key in (
                        ("base_params_file", "base"),
                        ("lidar_params_file", "lidar"),
                        ("nav2_params_file", "nav2"),
                    )
                }
                profile = load_profile(self.config["platform_file"], overrides)
                validate_profile(profile, "localization")
                validate_map_file(str(map_path))
        if not self.supervisor.process and mode != "idle":
            await self.wait_for_robot_exit()
        self.transitioning = True
        self.mode_id = secrets.token_hex(8)
        self.gate.mode = "idle"
        self.gate.stop()
        try:
            with suppress(ValueError):
                await self.navigator.cancel()
            self.bridge.publish_velocity((0, 0, 0))
            await self.supervisor.stop()
            # DDS discovery can briefly retain nodes after launch has reaped
            # every process. Drain it before clearing data or starting a new map.
            await self.wait_for_robot_exit()
            # After the owned ROS graph has exited, old action requests cannot
            # issue motion. Start a clean action state for the next graph.
            self.navigator = Navigator(self.bridge, self.gate)
            self.bridge.reset_data()
            await self.supervisor.start(mode, name, map_path)
            self.gate.mode = mode
        finally:
            self.transitioning = False

    async def wait_for_robot_exit(self):
        deadline = time.monotonic() + 4.0
        while True:
            names = self.bridge.get_node_names()
            if not any(
                node in names
                for node in ("kiwi_base_node", "tracked_base_node", "slam_toolbox", "controller_server", "amcl")
            ):
                return
            if time.monotonic() >= deadline:
                raise ValueError("检测到机器人节点仍在运行，请先停止其他启动入口后重试")
            await asyncio.sleep(0.1)

    async def command(self, request):
        body = await request.json()
        if not isinstance(body, dict):
            raise ValueError("请求格式无效")
        operation = body.get("op")
        # Immediate stop bypasses potentially slow launch/map operations.
        if operation == "estop":
            self.gate.stop(latch=True)
            self.bridge.publish_velocity((0, 0, 0))
            asyncio.create_task(self.cancel_safely())
            return web.json_response({"ok": True})
        if self.operation.locked():
            raise web.HTTPConflict(text="设备正在处理另一操作，请稍后重试")
        async with self.operation:
            result = await self.dispatch(operation, body)
        return web.json_response({"ok": True, "result": result})

    async def dispatch(self, operation, body):
        if operation in (
            "initial_pose",
            "navigate",
            "pause",
            "resume",
            "skip",
            "cancel",
            "clear_costmaps",
        ):
            if body.get("mode_id") != self.mode_id or self.transitioning:
                raise ValueError("运行模式或地图已改变，请等待页面更新后重新操作")
        if operation == "mode":
            await self.change_mode(body["mode"], body.get("map", ""))
        elif operation == "reset_estop":
            await self.navigator.cancel()
            self.bridge.clear_motion_fault()
            self.gate.reset()
        elif operation == "save_map":
            if self.supervisor.mode != "mapping" or not self.bridge.status()["mapping_ready"]:
                raise ValueError("请先完成有效建图")
            if any(abs(v) > 0.02 for v in self.bridge.status()["speed"]) or any(self.gate.output()):
                raise ValueError("请先松开控制并等待底盘静止后保存")
            self.gate.release()
            info, data = self.bridge.grid()
            return await asyncio.to_thread(self.store.save_grid, body["name"], info, data)
        elif operation == "edit_map":
            if self.supervisor.mode != "idle":
                raise ValueError("修图前请先停止运行模式")
            return await asyncio.to_thread(
                self.store.edit, body["name"], body["revision"], body["operations"]
            )
        elif operation == "delete_map":
            if self.supervisor.map_name == body["name"]:
                raise ValueError("不能删除正在使用的地图，请先停止导航")
            return await asyncio.to_thread(self.store.delete, body["name"])
        elif operation == "initial_pose":
            if self.supervisor.mode != "navigation":
                raise ValueError("请先加载导航地图")
            await self.navigator.cancel()
            self.bridge.initial_pose(pose(body["pose"]))
        elif operation == "navigate":
            points = body["points"]
            if not isinstance(points, list) or not 1 <= len(points) <= 200:
                raise ValueError("目标点数量须为 1–200")
            await self.navigator.start([pose(p) for p in points])
        elif operation == "pause":
            await self.navigator.pause()
        elif operation == "resume":
            await self.navigator.resume()
        elif operation == "skip":
            await self.navigator.skip()
        elif operation == "cancel":
            await self.navigator.cancel()
        elif operation == "clear_costmaps":
            if self.supervisor.mode != "navigation":
                raise ValueError("请先启动导航")
            await self.bridge.clear_costmaps()
        elif operation == "save_route":
            await asyncio.to_thread(
                self.store.save_route, body["map"], body["name"], body["points"]
            )
        elif operation == "delete_route":
            await asyncio.to_thread(self.store.delete_route, body["map"], body["name"])
        else:
            raise ValueError("未知操作")

    async def import_map(self, request):
        if self.operation.locked():
            raise web.HTTPConflict(text="设备正忙")
        async with self.operation:
            reader = await request.multipart()
            fields = {}
            total = 0
            async for part in reader:
                if part.name not in ("name", "yaml", "image") or part.name in fields:
                    raise ValueError("导入字段无效")
                data = bytearray()
                while True:
                    chunk = await part.read_chunk()
                    if not chunk:
                        break
                    total += len(chunk)
                    if total > 50 * 1024 * 1024:
                        raise ValueError("导入文件超过 50 MB")
                    data.extend(chunk)
                fields[part.name] = bytes(data)
            result = await asyncio.to_thread(
                self.store.import_map, fields["name"].decode(), fields["yaml"], fields["image"]
            )
            return web.json_response(result)

    async def websocket(self, request):
        if request.headers.get("Origin") != request.scheme + "://" + request.host:
            raise web.HTTPForbidden(text="WebSocket 必须同源")
        if len(self.clients) >= self.config.get("max_clients", 8):
            raise web.HTTPServiceUnavailable(text="连接数已满")
        socket = web.WebSocketResponse(heartbeat=2, max_msg_size=4096, compress=False)
        await socket.prepare(request)
        client_id = secrets.token_hex(8)
        self.clients[client_id] = socket
        await socket.send_json(dict(type="hello", client_id=client_id))
        sender = asyncio.create_task(self.send_state(socket))
        try:
            async for message in socket:
                if message.type != WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(message.data)
                    if not isinstance(data, dict):
                        raise ValueError("控制请求必须是对象")
                    operation = data.get("op")
                    if operation == "acquire":
                        if self.transitioning or self.operation.locked():
                            raise ValueError("模式切换中，请稍后重试")
                        self.gate.acquire(client_id)
                    elif operation == "drive":
                        self.gate.drive(client_id, data["velocity"], data["seq"], data["challenge"])
                    elif operation == "release":
                        self.gate.release(client_id)
                    elif operation == "estop":
                        self.gate.stop(latch=True)
                        asyncio.create_task(self.cancel_safely())
                    elif operation == "ping":
                        await socket.send_json(dict(type="pong", sent=data.get("sent")))
                    else:
                        raise ValueError("未知控制操作")
                    if operation in ("release", "estop", "drive"):
                        self.bridge.publish_velocity(self.gate.output())
                    if operation == "acquire":
                        await socket.send_json(dict(type="acquired"))
                except (ValueError, TypeError, KeyError) as error:
                    self.gate.release(client_id)
                    await socket.send_json(dict(type="error", message=str(error)))
        finally:
            self.gate.release(client_id)
            self.bridge.publish_velocity(self.gate.output())
            sender.cancel()
            await asyncio.gather(sender, return_exceptions=True)
            self.clients.pop(client_id, None)
        return socket

    async def send_state(self, socket):
        try:
            while not socket.closed:
                state = self.snapshot()
                state.update(type="state", challenge=self.gate.challenge())
                await asyncio.wait_for(socket.send_json(state), 0.3)
                await asyncio.sleep(0.1)
        except (ConnectionError, asyncio.TimeoutError):
            await socket.close()
        except Exception:
            LOG.exception("WebSocket state delivery failed")
            await socket.close(code=1011, message=b"State delivery failed")

    async def cancel_safely(self):
        async with self.operation:
            with suppress(ValueError):
                await self.navigator.cancel()

    async def maintenance(self):
        while True:
            self.supervisor.check()
            if self.supervisor.mode == "idle" and self.gate.mode != "idle":
                self.gate.mode = "idle"
                self.gate.stop(latch=True)
                await self.cancel_safely()
            fault = self.bridge.status().get("motion_fault")
            if fault and self.navigator.state == "running":
                async with self.operation:
                    with suppress(ValueError):
                        await self.navigator.cancel("failed")
                    self.navigator.message = fault
            self.system = dict(
                cpu=psutil.cpu_percent(),
                memory=psutil.virtual_memory().percent,
                disk=psutil.disk_usage(str(self.store.root)).percent,
                clients=len(self.clients),
            )
            await asyncio.sleep(1)

    async def static(self, request):
        name = request.match_info["path"] or "mapping.html"
        # colcon --symlink-install deliberately points installed assets into src.
        # Permit those trusted asset symlinks, but never a client-supplied path.
        path = self.web_root / name
        if Path(name).name != name or name.startswith(".") or not path.is_file():
            raise web.HTTPNotFound()
        response = web.FileResponse(path)
        response.headers["Cache-Control"] = "no-cache"
        response.headers["Content-Security-Policy"] = (
            "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; "
            "img-src 'self' blob: data:; connect-src 'self'; object-src 'none'; frame-ancestors 'none'"
        )
        return response

    async def startup(self, app):
        self.maintenance_task = asyncio.create_task(self.maintenance())

    async def shutdown(self, app):
        self.gate.stop(latch=True)
        self.bridge.publish_velocity((0, 0, 0))
        self.maintenance_task.cancel()
        await asyncio.gather(self.maintenance_task, return_exceptions=True)
        for socket in list(self.clients.values()):
            await socket.close(code=1001, message=b"Server shutting down")
        with suppress(ValueError):
            await self.navigator.cancel()
        await self.supervisor.stop()


def load_config(args):
    share = Path(get_package_share_directory("kiwi_web"))
    config = yaml.safe_load((share / "config/web.yaml").read_text())
    if args.config:
        config.update(yaml.safe_load(Path(args.config).read_text()) or {})
    for key in ("host", "port", "maps_dir", "workspace", "platform_file", *PARAMETER_FILES):
        if getattr(args, key, None) is not None:
            config[key] = getattr(args, key)
    resolve_paths(config)
    profile = load_profile(config.get("platform_file", ""),
                           {"base": config.get("base_params_file", "")})
    validate_profile(profile, "driver")
    config["platform_file"] = str(profile.selector)
    config["active_chassis"] = profile.chassis
    base_file = config.get("base_params_file") or profile.path("base")
    base = yaml.safe_load(Path(base_file).read_text())[profile.driver]["ros__parameters"]
    config["feedback_topic"] = (
        "/chassis/odom/laser" if profile.odometry == "rf2o" else base["wheel_odom_topic"]
    )
    config["imu_topic"] = base["imu_topic"] if profile.odometry == "rf2o" else ""
    config["imu_frame_id"] = base["imu_frame_id"] if profile.odometry == "rf2o" else ""
    config["motion_enabled"] = (
        base["enable_motion"] and base["hardware_calibrated"]
        if profile.odometry == "rf2o" else True
    )
    limits = []
    for web_key, base_key in (
        ("manual_linear", "max_linear_speed_m_s"),
        ("manual_lateral", "max_lateral_speed_m_s"),
        ("manual_angular", "max_angular_speed_rad_s"),
    ):
        value = finite(config[web_key])
        if value <= 0:
            raise ValueError("速度上限必须大于零")
        limits.append(min(value, finite(base.get(base_key, 0.0))))
    navigation_limits = tuple(
        finite(base.get(key, 0.0))
        for key in ("max_linear_speed_m_s", "max_lateral_speed_m_s", "max_angular_speed_rad_s")
    )
    if navigation_limits[0] <= 0 or navigation_limits[2] <= 0:
        raise ValueError("底盘前后与旋转速度上限必须大于零")
    return config, limits, navigation_limits, share


def main():
    parser = argparse.ArgumentParser(description="Kiwi local web console")
    parser.add_argument("--config")
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--maps-dir")
    parser.add_argument("--workspace")
    parser.add_argument("--platform-file")
    for key in PARAMETER_FILES:
        parser.add_argument("--" + key.replace("_", "-"))
    args, ros_args = parser.parse_known_args()
    logging.basicConfig(level=logging.INFO)
    config, limits, navigation_limits, share = load_config(args)
    domain = int(os.environ.get("ROS_DOMAIN_ID", config.get("ros_domain_id", 42)))
    if not 0 <= domain <= 232:
        raise ValueError("ROS_DOMAIN_ID 必须为 0–232")
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    # One gate per ROS domain on this device, even with another HTTP port.
    lock_root = Path(os.environ.get("XDG_RUNTIME_DIR", "/tmp"))
    lock_path = lock_root / (
        "kiwi_web_%s_%s.lock" % (os.getuid(), os.environ.get("ROS_DOMAIN_ID", "0"))
    )
    lock_file = lock_path.open("a")
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        raise RuntimeError("此 ROS 域已有 Web 控制服务在运行") from error
    rclpy.init(args=ros_args)
    gate = CommandGate(*limits, navigation_limits=navigation_limits)
    bridge = RosBridge(
        gate, feedback_topic=config["feedback_topic"],
        imu_topic=config["imu_topic"], imu_frame_id=config["imu_frame_id"],
        motion_enabled=config["motion_enabled"],
    )
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    console = Console(
        config, bridge, gate, MapStore(config["maps_dir"]), Supervisor(config), share / "web"
    )
    try:
        web.run_app(console.app, host=config["host"], port=config["port"], access_log=None)
    finally:
        gate.stop(latch=True)
        bridge.publish_velocity((0, 0, 0))
        executor.shutdown()
        thread.join(timeout=3)
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock_file.close()
