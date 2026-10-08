# Copyright 2026 Kiwi contributors
# Licensed under the MIT License. See ../LICENSE.txt.
"""Exercise the embedded SDK and CustomMsg without starting lidar hardware."""

import ctypes
import os
from pathlib import Path
import re
import subprocess

from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.serialization import deserialize_message, serialize_message


def test_driver_has_no_external_sdk_dependency():
    result = subprocess.run(
        ['ldd', os.environ['LIVOX_DRIVER_LIBRARY']],
        check=True, capture_output=True, text=True,
    )
    assert 'not found' not in result.stdout
    assert 'liblivox_lidar_sdk_shared' not in result.stdout


def test_embedded_sdk_version_api():
    class SdkVersion(ctypes.Structure):
        _fields_ = [('major', ctypes.c_int), ('minor', ctypes.c_int),
                    ('patch', ctypes.c_int)]

    driver = ctypes.CDLL(os.environ['LIVOX_DRIVER_LIBRARY'])
    driver.GetLivoxLidarSdkVer.argtypes = [ctypes.POINTER(SdkVersion)]
    driver.GetLivoxLidarSdkVer.restype = None
    version = SdkVersion()
    driver.GetLivoxLidarSdkVer(ctypes.byref(version))
    header = Path(os.environ['LIVOX_SDK_HEADER']).read_text()
    expected = tuple(int(re.search(
        rf'#define\s+LIVOX_LIDAR_SDK_{part}_VERSION\s+(\d+)', header,
    ).group(1)) for part in ('MAJOR', 'MINOR', 'PATCH'))
    assert (version.major, version.minor, version.patch) == expected


def test_custom_msg_typesupport_roundtrip():
    point = CustomPoint(offset_time=1000, x=1.25, y=-2.5, z=0.5,
                        reflectivity=42, tag=16, line=2)
    message = CustomMsg(timebase=123456789, point_num=1, lidar_id=117,
                        points=[point])
    restored = deserialize_message(serialize_message(message), CustomMsg)
    assert restored == message
