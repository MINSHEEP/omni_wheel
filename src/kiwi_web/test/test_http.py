"""Exercise real HTTP/WebSocket transport without starting physical hardware."""

import asyncio
from pathlib import Path
import threading

from aiohttp import ClientSession, web

from kiwi_web.control import CommandGate
from kiwi_web.maps import MapStore
from kiwi_web.server import Console


class Bridge:
    def __init__(self):
        self.commands = []
        self.lock = threading.RLock()
        self.path = []

    def publish_velocity(self, value):
        self.commands.append(value)

    def status(self):
        return dict(speed=[0, 0, 0], mapping_ready=True)


class Supervisor:
    mode = "idle"
    map_name = ""
    error = ""
    log = []
    process = None

    def check(self):
        pass

    async def stop(self):
        pass


async def message(socket, kind):
    for _ in range(20):
        data = await socket.receive_json(timeout=3)
        if data["type"] == kind:
            return data
    raise AssertionError("No " + kind)


def test_http_map_revision_and_websocket_ownership(tmp_path):
    async def exercise():
        gate = CommandGate()
        bridge, supervisor = Bridge(), Supervisor()
        store = MapStore(tmp_path)
        store.save_grid(
            "lab", dict(width=2, height=2, resolution=0.1, origin=[0, 0, 0]), [0, 0, 100, -1]
        )
        console = Console(
            dict(port=8080),
            bridge,
            gate,
            store,
            supervisor,
            Path(__file__).resolve().parents[1] / "web",
        )
        runner = web.AppRunner(console.app)
        await runner.setup()
        site = web.TCPSite(runner, "127.0.0.1", 0)
        await site.start()
        base = "http://127.0.0.1:" + str(site._server.sockets[0].getsockname()[1])
        try:
            async with ClientSession() as client:
                async with client.get(base + "/") as response:
                    assert response.status == 200
                    assert "二维建图" in await response.text()
                async with client.post(base + "/api/command", json={"op": "estop"}) as response:
                    assert response.status == 403
                async with client.post(
                    base + "/api/command",
                    json={"op": "estop"},
                    headers={"X-Kiwi-Request": "1", "Origin": "http://evil.invalid"},
                ) as response:
                    assert response.status == 403
                document = await (await client.get(base + "/api/maps/lab")).json()
                request = dict(
                    op="edit_map",
                    name="lab",
                    revision=document["revision"],
                    operations=[dict(value=0, indices=[0])],
                )
                headers = {"X-Kiwi-Request": "1"}
                async with client.post(
                    base + "/api/command",
                    json=dict(op="navigate", mode_id="old-mode", points=[dict(x=0, y=0, yaw=0)]),
                    headers=headers,
                ) as response:
                    assert response.status == 400
                    assert "地图已改变" in (await response.json())["error"]
                async with client.post(
                    base + "/api/command", json=request, headers=headers
                ) as response:
                    assert response.status == 200
                async with client.post(
                    base + "/api/command", json=request, headers=headers
                ) as response:
                    assert response.status == 400
                    assert "其他页面" in (await response.json())["error"]
                supervisor.mode = gate.mode = "remote"
                gate.manual_ready = True
                async with client.ws_connect(base + "/ws", origin=base) as first:
                    owner = (await message(first, "hello"))["client_id"]
                    challenge = (await message(first, "state"))["challenge"]
                    await first.send_json({"op": "acquire"})
                    await message(first, "acquired")
                    await first.send_json([])
                    assert "必须是对象" in (await message(first, "error"))["message"]
                    await first.send_json({"op": "acquire"})
                    await message(first, "acquired")
                    await first.send_json(
                        dict(op="drive", velocity=[0.2, 0.1, 0], seq=0, challenge=challenge)
                    )
                    await asyncio.sleep(0.04)
                    assert gate.output() == (0.2, 0.1, 0)
                    async with client.ws_connect(base + "/ws", origin=base) as second:
                        await second.send_json({"op": "acquire"})
                        assert "另一浏览器" in (await message(second, "error"))["message"]
                    assert gate.status()["owner"] == owner
                    await first.send_json(
                        dict(op="drive", velocity=[0.3, 0, 0], seq=1, challenge="stale")
                    )
                    assert "过期" in (await message(first, "error"))["message"]
                    assert gate.output() == (0, 0, 0)
                    await first.send_json({"op": "acquire"})
                    await message(first, "acquired")
                await asyncio.sleep(0.03)
                assert gate.status()["owner"] is None
                async with client.post(
                    base + "/api/command", json={"op": "estop"}, headers=headers
                ) as response:
                    assert response.status == 200
                assert gate.estop
        finally:
            await runner.cleanup()

    asyncio.run(exercise())


def test_authentication_requires_password_and_same_origin(tmp_path):
    async def exercise():
        console = Console(
            dict(port=8080, access_token="test-password"),
            Bridge(),
            CommandGate(),
            MapStore(tmp_path),
            Supervisor(),
            Path(__file__).resolve().parents[1] / "web",
        )
        runner = web.AppRunner(console.app)
        await runner.setup()
        site = web.TCPSite(runner, "127.0.0.1", 0)
        await site.start()
        base = "http://127.0.0.1:" + str(site._server.sockets[0].getsockname()[1])
        try:
            async with ClientSession() as client:
                assert (await client.get(base + "/api/maps")).status == 401
                response = await client.post(
                    base + "/api/login",
                    json={"token": "incorrect"},
                    headers={"X-Kiwi-Request": "1"},
                )
                assert response.status == 401
                response = await client.post(
                    base + "/api/login",
                    json={"token": "test-password"},
                    headers={"X-Kiwi-Request": "1"},
                )
                assert response.status == 200
                cookie = response.cookies["kiwi_session"].value
                response = await client.get(base + "/api/maps", cookies={"kiwi_session": cookie})
                assert response.status == 200
        finally:
            await runner.cleanup()

    asyncio.run(exercise())
