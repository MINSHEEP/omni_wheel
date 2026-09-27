"""Web map save must persist the MID360 PCD and 2D map together."""

import asyncio
from pathlib import Path
import struct

import pytest

from kiwi_web.control import CommandGate
from kiwi_web.maps import MapStore
from kiwi_web.server import Console


PCD_HEADER = (
    b"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
    b"COUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary\n"
)


class Bridge:
    def __init__(self, stage, *, write_pcd=True):
        self.stage = stage
        self.write_pcd = write_pcd
        self.calls = 0

    def status(self):
        return {"mapping_ready": True, "speed": [0.0, 0.0, 0.0]}

    def publish_velocity(self, velocity):
        assert tuple(velocity) == (0, 0, 0)

    def grid(self):
        return ({"width": 2, "height": 2, "resolution": 0.1,
                 "origin": [0.0, 0.0, 0.0]}, [0, 100, -1, 0])

    async def save_pointcloud(self):
        self.calls += 1
        if self.write_pcd:
            self.stage.write_bytes(PCD_HEADER + struct.pack("<3f", 1.0, 2.0, 3.0))


class Supervisor:
    mode = "mapping"
    map_name = ""
    error = ""
    log = []

    def __init__(self, stage):
        self.session_pcd = str(stage)
        self.session_pcd_owned = True


def console(tmp_path, *, write_pcd=True):
    stage = tmp_path / ".session.pcd"
    bridge = Bridge(stage, write_pcd=write_pcd)
    store = MapStore(tmp_path)
    service = Console(
        {"port": 8080, "sensor_pipeline": "mid360s"}, bridge, CommandGate(),
        store, Supervisor(stage), Path(__file__).resolve().parents[1] / "web",
    )
    return service, bridge, store


def test_mid360_map_save_persists_matching_pcd_and_2d_map(tmp_path):
    service, bridge, store = console(tmp_path)
    result = asyncio.run(service.dispatch("save_map", {"name": "hall"}))
    assert result["pcd"]["points"] == 1
    assert bridge.calls == 1
    assert store.path("hall").is_file()
    assert (tmp_path / "hall.pcd").is_file()
    assert store.read("hall")["width"] == 2


def test_missing_pcd_does_not_claim_2d_map_saved(tmp_path):
    service, bridge, store = console(tmp_path, write_pcd=False)
    with pytest.raises(ValueError, match="未生成点云"):
        asyncio.run(service.dispatch("save_map", {"name": "hall"}))
    assert bridge.calls == 1
    assert not store.path("hall").exists()
    assert not (tmp_path / "hall.pcd").exists()


def test_existing_map_rejected_before_pointcloud_save(tmp_path):
    service, bridge, store = console(tmp_path)
    asyncio.run(service.dispatch("save_map", {"name": "hall"}))
    with pytest.raises(ValueError, match="已存在"):
        asyncio.run(service.dispatch("save_map", {"name": "hall"}))
    assert bridge.calls == 1


def test_deleting_2d_map_archives_its_matching_pointcloud(tmp_path):
    service, _, store = console(tmp_path)
    asyncio.run(service.dispatch("save_map", {"name": "hall"}))
    store.delete("hall")
    assert not (tmp_path / "hall.pcd").exists()
    assert len(list((tmp_path / ".versions").glob("hall_deleted_*.pcd"))) == 1
