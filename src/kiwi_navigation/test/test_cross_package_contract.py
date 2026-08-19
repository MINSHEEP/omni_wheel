"""Nav2、底盘与 SLAM 之间的安全配置契约."""

from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE.parent
NAV2_CONFIG = PACKAGE / "config" / "nav2.yaml"
BASE_CONFIG = SOURCE_ROOT / "kiwi_base" / "config" / "kiwi_base.yaml"
SLAM_CONFIG_DIR = SOURCE_ROOT / "kiwi_slam" / "config"


def load_yaml(path: Path):
    """以 UTF-8 解析源码树中的 ROS 参数文件."""
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def node_parameters(config, node: str):
    """取出标准 ROS 2 YAML 节点的参数映射."""
    return config[node]["ros__parameters"]


def costmap_parameters(config, scope: str):
    """取出 Nav2 局部或全局 costmap 参数."""
    return config[scope][scope]["ros__parameters"]


def assert_magnitudes_within(values, limit: float):
    """正、负方向上限都不得突破硬件约束."""
    assert limit > 0.0
    assert all(abs(float(value)) <= limit for value in values)


def test_nav2_velocity_and_acceleration_limits_fit_base_hardware():
    """DWB 与平滑器的三轴限值必须收敛到底盘物理上限内."""
    nav2 = load_yaml(NAV2_CONFIG)
    base = node_parameters(load_yaml(BASE_CONFIG), "kiwi_base_node")
    controller = node_parameters(nav2, "controller_server")
    dwb = controller["FollowPath"]
    smoother = node_parameters(nav2, "velocity_smoother")

    # 0x3A 命令没有 ACK；速度平滑器保持开环，不得把异步 0x3B 遥测当作应答门控。
    assert smoother["feedback"] == "OPEN_LOOP"

    speed_limits = (
        base["max_linear_speed_m_s"],
        base["max_lateral_speed_m_s"],
        base["max_angular_speed_rad_s"],
    )
    acceleration_limits = (
        base["max_linear_acceleration_m_s2"],
        base["max_lateral_acceleration_m_s2"],
        base["max_angular_acceleration_rad_s2"],
    )

    assert_magnitudes_within((dwb["min_vel_x"], dwb["max_vel_x"]), speed_limits[0])
    assert_magnitudes_within((dwb["min_vel_y"], dwb["max_vel_y"]), speed_limits[1])
    assert_magnitudes_within((dwb["max_vel_theta"],), speed_limits[2])
    assert_magnitudes_within((dwb["rotate_to_heading_angular_vel"],), speed_limits[2])
    assert_magnitudes_within((dwb["max_speed_xy"],), max(speed_limits[:2]))
    for axis, limit in enumerate(speed_limits):
        assert_magnitudes_within(
            (smoother["min_velocity"][axis], smoother["max_velocity"][axis]),
            limit,
        )

    dwb_accelerations = (
        (dwb["acc_lim_x"], dwb["decel_lim_x"]),
        (dwb["acc_lim_y"], dwb["decel_lim_y"]),
        (dwb["acc_lim_theta"], dwb["decel_lim_theta"]),
    )
    for axis, limit in enumerate(acceleration_limits):
        assert_magnitudes_within(dwb_accelerations[axis], limit)
        assert_magnitudes_within(
            (smoother["max_accel"][axis], smoother["max_decel"][axis]),
            limit,
        )

    # RotationShim 和 Spin 与平滑器共用同一角加速度契约。
    assert_magnitudes_within((dwb["max_angular_accel"],), acceleration_limits[2])
    behavior = node_parameters(nav2, "behavior_server")
    assert_magnitudes_within(
        (behavior["min_rotational_vel"], behavior["max_rotational_vel"]),
        speed_limits[2],
    )
    assert dwb["max_angular_accel"] == smoother["max_accel"][2]
    assert behavior["rotational_acc_lim"] == smoother["max_accel"][2]


def test_local_and_global_costmap_parameters_are_synchronized():
    """局部与全局 costmap 不得对同一雷达观测作出不同解释."""
    nav2 = load_yaml(NAV2_CONFIG)
    local = costmap_parameters(nav2, "local_costmap")
    global_ = costmap_parameters(nav2, "global_costmap")

    for key in ("inflation_radius", "cost_scaling_factor"):
        assert local["inflation_layer"][key] == global_["inflation_layer"][key]

    for key in ("plugin", "enabled", "observation_sources"):
        assert local["obstacle_layer"][key] == global_["obstacle_layer"][key]
    # 比较 scan 映射，同时覆盖 Inf、marking 与 clearing。obstacle_max_range
    # 是唯一允许的差异：局部 6 m 滚动窗中心到角落仅 4.24 m，超窗标记全部
    # 丢弃，因此局部量程可以更小，但不得超过全局量程（全局提供规划前瞻）。
    local_scan = dict(local["obstacle_layer"]["scan"])
    global_scan = dict(global_["obstacle_layer"]["scan"])
    assert local_scan.pop("obstacle_max_range") <= global_scan.pop("obstacle_max_range")
    assert local_scan == global_scan
    assert local["obstacle_layer"]["scan"]["inf_is_valid"] is False


def test_robot_radius_covers_physical_wheel_outer_edge():
    """Nav2 碰撞半径必须覆盖轮心距加半轮宽形成的外接圆."""
    nav2 = load_yaml(NAV2_CONFIG)
    base_config = load_yaml(BASE_CONFIG)
    base = node_parameters(base_config, "kiwi_base_node")
    model = node_parameters(base_config, "kiwi_robot_model")
    physical_radius = base["wheel_center_distance_m"] + model["wheel_width_m"] / 2.0

    local_radius = costmap_parameters(nav2, "local_costmap")["robot_radius"]
    global_radius = costmap_parameters(nav2, "global_costmap")["robot_radius"]
    assert local_radius == global_radius
    assert local_radius >= physical_radius


def test_tf_and_scan_names_match_across_packages():
    """各包必须共用同一组 Frame 和扫描话题."""
    base_config = load_yaml(BASE_CONFIG)
    base = node_parameters(base_config, "kiwi_base_node")
    ekf = node_parameters(base_config, "ekf_filter_node")
    monitor = node_parameters(
        load_yaml(SLAM_CONFIG_DIR / "monitor.yaml"), "kiwi_slam_monitor"
    )
    lidar = node_parameters(
        load_yaml(SLAM_CONFIG_DIR / "ls_m10p_uart.yaml"), "lslidar_driver_node"
    )
    slam = node_parameters(
        load_yaml(SLAM_CONFIG_DIR / "slam_toolbox.yaml"), "slam_toolbox"
    )

    assert {monitor["laser_frame"], lidar["frame_id"]} == {"lidar_link"}
    assert {
        monitor["odom_frame"],
        slam["odom_frame"],
        base["odom_frame_id"],
        ekf["odom_frame"],
    } == {"odom"}
    assert {
        monitor["base_frame"],
        slam["base_frame"],
        base["base_frame_id"],
        ekf["base_link_frame"],
    } == {"base_footprint"}
    assert {monitor["scan_topic"], lidar["laserscan_topic"], slam["scan_topic"]} == {
        "/scan"
    }
