"""PCD file validation and durable named map copy contracts."""

import os
import struct

import pytest

from kiwi_web.pointcloud import copy_validated_pcd, point_cloud_path, validate_pcd


def pcd_header(encoding="binary", points=2):
    return (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {points}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {points}\n"
        f"DATA {encoding}\n"
    ).encode("ascii")


def test_binary_pcd_is_copied_and_cannot_overwrite(tmp_path):
    source = tmp_path / "live.pcd"
    source.write_bytes(pcd_header() + struct.pack("<8f", 0, 0, 0, 1, 1, 2, 3, 0.5))
    destination = tmp_path / "maps"
    metadata = copy_validated_pcd(source, destination, "office_a")
    saved = point_cloud_path(destination, "office_a")
    assert saved.read_bytes() == source.read_bytes()
    assert metadata == {"points": 2, "encoding": "binary", "size_bytes": saved.stat().st_size}
    with pytest.raises(ValueError, match="已存在"):
        copy_validated_pcd(source, destination, "office_a")
    assert saved.read_bytes() == source.read_bytes()
    assert list(destination.glob("*.tmp")) == []


def test_ascii_and_compressed_pcd_structural_checks(tmp_path):
    ascii_pcd = tmp_path / "ascii.pcd"
    ascii_pcd.write_bytes(pcd_header("ascii") + b"0 0 0 1\n1 2 3 0.5\n")
    assert validate_pcd(ascii_pcd)["points"] == 2
    ascii_pcd.write_bytes(pcd_header("ascii") + b"0 0 0 1\n")
    with pytest.raises(ValueError, match="ASCII"):
        validate_pcd(ascii_pcd)

    compressed = tmp_path / "compressed.pcd"
    compressed.write_bytes(pcd_header("binary_compressed") + struct.pack("<II", 5, 32) + b"abcde")
    assert validate_pcd(compressed)["encoding"] == "binary_compressed"
    compressed.write_bytes(pcd_header("binary_compressed") + struct.pack("<II", 6, 32) + b"abcde")
    with pytest.raises(ValueError, match="压缩数据"):
        validate_pcd(compressed)


def test_rejects_truncated_binary_invalid_header_and_unsafe_names(tmp_path):
    source = tmp_path / "live.pcd"
    source.write_bytes(pcd_header() + b"\x00" * 16)
    with pytest.raises(ValueError, match="二进制数据长度"):
        validate_pcd(source)
    with pytest.raises(ValueError, match="二进制数据长度"):
        copy_validated_pcd(source, tmp_path / "maps", "lab")
    assert not (tmp_path / "maps/lab.pcd").exists()

    source.write_bytes(pcd_header(points=0))
    with pytest.raises(ValueError, match="大于零"):
        validate_pcd(source)
    for name in ("../escape", "/tmp/unsafe", "a.b", ""):
        with pytest.raises(ValueError, match="名称"):
            point_cloud_path(tmp_path / "maps", name)


def test_rejects_symlink_source_and_destination(tmp_path):
    source = tmp_path / "live.pcd"
    source.write_bytes(pcd_header(points=1) + struct.pack("<4f", 0, 0, 0, 1))
    maps = tmp_path / "maps"
    maps.mkdir()
    link = tmp_path / "linked.pcd"
    link.symlink_to(source)
    with pytest.raises(ValueError, match="符号链接"):
        copy_validated_pcd(link, maps, "lab")
    (maps / "lab.pcd").symlink_to(source)
    with pytest.raises(ValueError, match="符号链接"):
        copy_validated_pcd(source, maps, "lab")
    assert os.path.islink(maps / "lab.pcd")
