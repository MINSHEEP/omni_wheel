"""Manage only the ROS launch process group owned by this web instance."""

import asyncio
from collections import deque
import os
import signal

from .config import PARAMETER_FILES


class Supervisor:
    def __init__(self, config):
        self.config = config
        self.process = None
        self.reader = None
        self.mode = "idle"
        self.map_name = ""
        self.log = deque(maxlen=100)
        self.error = ""

    async def start(self, mode, map_name="", map_path=""):
        if (
            mode == self.mode
            and map_name == self.map_name
            and self.process
            and self.process.returncode is None
        ):
            return
        await self.stop()
        if mode == "idle":
            return
        # Use the installed ROS command, never a shell or client-provided executable.
        command = ["ros2", "launch", "kiwi_web", "stack.launch.py", "mode:=" + mode]
        command.append("platform_file:=" + self.config.get("platform_file", ""))
        command.append("expected_chassis:=" + self.config.get("active_chassis", ""))
        if map_path:
            command.append("map:=" + str(map_path))
        for key in PARAMETER_FILES:
            if self.config.get(key):
                command.append(key + ":=" + self.config[key])
        self.error = ""
        self.log.clear()
        self.process = await asyncio.create_subprocess_exec(
            *command,
            start_new_session=True,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
            limit=1024 * 1024
        )
        self.mode, self.map_name = mode, map_name
        self.reader = asyncio.create_task(self._read(self.process))
        await asyncio.sleep(0.2)
        self.check()
        if self.error:
            raise ValueError(self.error)

    async def _read(self, process):
        while True:
            line = await process.stdout.readline()
            if not line:
                break
            self.log.append(line.decode(errors="replace").strip()[-1000:])

    def check(self):
        if self.process and self.process.returncode is not None and self.mode != "idle":
            self.error = "ROS 进程已退出：" + str(self.process.returncode)
            self.mode = "idle"
            self.map_name = ""

    async def stop(self):
        process = self.process
        self.mode, self.map_name = "idle", ""
        if process:
            # Signal the owned group even if its launch parent has already died.
            for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 1)):
                try:
                    if sig == signal.SIGINT and process.returncode is None:
                        # launch forwards SIGINT to children itself. Sending to
                        # the group here delivers it twice and can interrupt
                        # DDS cleanup, leaving ghost nodes until lease expiry.
                        process.send_signal(sig)
                    else:
                        os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    await asyncio.wait_for(process.wait(), timeout)
                    # launch normally waits for its children before exiting.
                    try:
                        os.killpg(process.pid, 0)
                    except ProcessLookupError:
                        break
                except asyncio.TimeoutError:
                    continue
            if self.reader:
                try:
                    await asyncio.wait_for(self.reader, 2)
                except asyncio.TimeoutError:
                    self.reader.cancel()
        self.process = None
