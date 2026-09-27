"""Render the Livox MID360 network configuration from real host/device addresses."""

import ipaddress
import json
import subprocess


def validate_network(interface, host_ip, lidar_ip):
    if not interface or not host_ip or not lidar_ip:
        raise ValueError("MID360 需要 lidar_interface、host_ip 和 lidar_ip 的实机值")
    host = ipaddress.IPv4Address(host_ip)
    lidar = ipaddress.IPv4Address(lidar_ip)
    if host.is_loopback or lidar.is_loopback or host == lidar:
        raise ValueError("MID360 主机与雷达必须使用不同的非回环 IPv4 地址")
    result = subprocess.run(["ip", "-j", "address", "show", "dev", interface],
                            capture_output=True, text=True, check=False)
    if result.returncode != 0:
        raise ValueError(f"找不到 MID360 网卡 {interface}: {result.stderr.strip()}")
    addresses = json.loads(result.stdout)
    if not addresses or not any(item.get("local") == str(host)
                                for item in addresses[0].get("addr_info", [])
                                if item.get("family") == "inet"):
        raise ValueError(f"{interface} 未配置 host_ip={host}")
    route = subprocess.run(["ip", "route", "get", str(lidar)], capture_output=True,
                           text=True, check=False)
    if route.returncode != 0 or f" dev {interface} " not in f" {route.stdout.strip()} ":
        raise ValueError(f"到 MID360 {lidar} 的路由不经过网卡 {interface}")
    return str(host), str(lidar)


def driver_config(host_ip, lidar_ip, device_model="Mid360s"):
    """Ports follow Livox SDK2; the deployed device advertises type 35 (Mid360s)."""
    if device_model not in ("MID360", "Mid360s"):
        raise ValueError("雷达型号必须是 MID360 或 Mid360s")
    return {
        "lidar_summary_info": {"lidar_type": 8},
        device_model: {
            "lidar_net_info": {
                "cmd_data_port": 56100, "push_msg_port": 56200,
                "point_data_port": 56300, "imu_data_port": 56400,
                "log_data_port": 56500,
            },
            # SDK2 新格式按雷达 IP 建立专用通道；旧版对象格式只等待广播发现，
            # 在已有设备持续向旧端口推流时可能始终不绑定点云与 IMU 端口。
            "host_net_info": [{
                "lidar_ip": [lidar_ip], "host_ip": host_ip, "multicast_ip": "",
                "cmd_data_port": 56101, "push_msg_port": 56201,
                "point_data_port": 56301, "imu_data_port": 56401,
                "log_data_port": 56501,
            }],
        },
        "lidar_configs": [{
            "ip": lidar_ip, "pcl_data_type": 1, "pattern_mode": 0,
            "extrinsic_parameter": {
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                "x": 0, "y": 0, "z": 0,
            },
        }],
    }
