#!/usr/bin/env python3
"""Start/stop the owned real stack with nonexistent serial ports in ROS domain 219."""

import asyncio, os, tempfile
from pathlib import Path
import yaml, psutil
from ament_index_python.packages import get_package_share_directory
from kiwi_web.supervisor import Supervisor

os.environ["ROS_DOMAIN_ID"] = "219"
os.environ["ROS_LOCALHOST_ONLY"] = "1"


async def main():
    with tempfile.TemporaryDirectory(prefix="kiwi-stack-test-") as directory:
        root = Path(directory)
        base = yaml.safe_load(
            (Path(get_package_share_directory("kiwi_base")) / "config/kiwi_base.yaml").read_text()
        )
        base["kiwi_base_node"]["ros__parameters"]["device"] = str(root / "absent-base-device")
        lidar = yaml.safe_load(
            (
                Path(get_package_share_directory("kiwi_slam")) / "config/ls_m10p_uart.yaml"
            ).read_text()
        )
        for section in lidar.values():
            if isinstance(section, dict) and "ros__parameters" in section:
                section["ros__parameters"]["serial_port"] = str(root / "absent-lidar-device")
        (root / "base.yaml").write_text(yaml.safe_dump(base))
        (root / "lidar.yaml").write_text(yaml.safe_dump(lidar))
        (root / "active_platform.yaml").write_text("chassis: kiwi\n")
        supervisor = Supervisor(
            dict(
                base_params_file=str(root / "base.yaml"), lidar_params_file=str(root / "lidar.yaml"),
                platform_file=str(root / "active_platform.yaml"), sensor_pipeline="legacy",
                active_chassis="kiwi",
            )
        )
        try:
            for mode in ["remote", "mapping"]:
                await supervisor.start(mode)
                await asyncio.sleep(3)
                assert supervisor.process.returncode is None, list(supervisor.log)
                parent = psutil.Process(supervisor.process.pid)
                children = parent.children(recursive=True)
                print(mode, "processes", [(p.pid, p.name()) for p in children], flush=True)
                assert any("kiwi_base_node" in p.name() for p in children)
                if mode == "mapping":
                    assert any("sync_slam_tool" in p.name() for p in children)
                pids = [(p.pid, p.create_time()) for p in [parent] + children]
                await supervisor.stop()
                alive = []
                for pid, born in pids:
                    try:
                        p = psutil.Process(pid)
                        if p.create_time() == born and p.status() != psutil.STATUS_ZOMBIE:
                            alive.append(pid)
                    except psutil.NoSuchProcess:
                        pass
                assert not alive, alive
                print(mode, "owned stack stopped without surviving children", flush=True)
        finally:
            await supervisor.stop()


if __name__ == "__main__":
    asyncio.run(main())
