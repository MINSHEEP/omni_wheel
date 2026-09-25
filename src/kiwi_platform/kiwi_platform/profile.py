"""从唯一选择文件解析底盘专属参数，并在启动硬件前核验整条链路。"""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

FILE_KEYS = ("base", "nav2", "slam", "gate", "lidar", "monitor", "urdf")
KNOWN_CHASSIS = {"kiwi", "tracked_pwm"}


def _read_yaml(path: Path) -> dict:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"无法读取配置 {path}: {error}") from error
    if not isinstance(data, dict):
        raise ValueError(f"配置必须是 YAML 映射：{path}")
    return data


def _node(data: dict, name: str, path: Path) -> dict:
    value = data.get(name, {}).get("ros__parameters")
    if not isinstance(value, dict):
        raise ValueError(f"{path} 缺少 {name}.ros__parameters")
    return value


def _equal(label: str, *values) -> None:
    if not all(value == values[0] for value in values[1:]):
        raise ValueError(f"参数链路不一致：{label} = {values}")


def _positive(name: str, value) -> None:
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} 必须是有限正数")


@dataclass(frozen=True)
class Profile:
    chassis: str
    driver: str
    odometry: str
    lidar: str
    imu: str
    selector: Path
    files: dict[str, Path]

    def path(self, key: str) -> str:
        return str(self.files[key])

    def data(self, key: str) -> dict:
        return _read_yaml(self.files[key])


def load_profile(selector_file: str = "", overrides: dict | None = None,
                 required_keys: tuple[str, ...] | None = None) -> Profile:
    """所有入口调用此函数；空 selector_file 使用安装包内唯一的活动选择文件。"""
    share = Path(get_package_share_directory("kiwi_platform"))
    selector = Path(selector_file).expanduser().resolve() if selector_file else share / "config/active_platform.yaml"
    selection = _read_yaml(selector)
    chassis = selection.get("chassis")
    if chassis not in KNOWN_CHASSIS:
        raise ValueError(f"{selector} 的 chassis 无效：{chassis!r}；可选 {sorted(KNOWN_CHASSIS)}")
    profile_path = share / "config/profiles" / f"{chassis}.yaml"
    config = _read_yaml(profile_path)
    if config.get("chassis") != chassis:
        raise ValueError(f"{profile_path} 的 chassis 与活动选择不一致")
    if config.get("driver") != ("kiwi_base_node" if chassis == "kiwi" else "tracked_base_node"):
        raise ValueError(f"{profile_path} 的底盘节点与 chassis 不匹配")
    if config.get("odometry") != ("wheel" if chassis == "kiwi" else "rf2o"):
        raise ValueError(f"{profile_path} 的里程计来源与 chassis 不匹配")
    if config.get("lidar") != "m10p" or config.get("imu") != "chassis":
        raise ValueError(f"{profile_path} 的雷达或惯导类型尚无对应启动实现")
    references = config.get("files")
    if not isinstance(references, dict):
        raise ValueError(f"{profile_path} 缺少 files 映射")
    available = FILE_KEYS + (("rf2o",) if chassis == "tracked_pwm" else ())
    keys = available if required_keys is None else required_keys
    if any(key not in available for key in keys):
        raise ValueError(f"{chassis} 请求了不存在的参数类别：{keys}")
    # 防止旧工作区同名包覆盖其中一环，导致底盘与 Nav2 读取不同版本的文件。
    def install_root(package: str) -> Path:
        prefix = Path(get_package_prefix(package)).resolve()
        return prefix.parent if prefix.name == package else prefix

    expected_root = install_root("kiwi_platform")
    files = {}
    for key in keys:
        ref = references.get(key)
        if not isinstance(ref, str) or "/" not in ref:
            raise ValueError(f"{profile_path} 的 files.{key} 必须是 包名/包内路径")
        package, relative = ref.split("/", 1)
        if package in ("kiwi_base", "kiwi_slam", "kiwi_navigation") and install_root(package) != expected_root:
            raise ValueError(f"{package} 与 kiwi_platform 来自不同工作区；请重新加载目标工作区")
        if relative.startswith("/") or ".." in Path(relative).parts:
            raise ValueError(f"{profile_path} 的 files.{key} 路径非法")
        files[key] = Path(get_package_share_directory(package)) / relative
    for key, value in (overrides or {}).items():
        if not value:
            continue
        if key not in files or key == "urdf":
            raise ValueError(f"不支持覆盖配置 {key}")
        files[key] = Path(value).expanduser().resolve()
    for key, path in files.items():
        if not path.is_file():
            raise ValueError(f"{chassis} 的 {key} 文件不存在：{path}")
    return Profile(chassis, config["driver"], config["odometry"], config["lidar"], config["imu"], selector, files)




def validate_lidar(profile: Profile) -> dict:
    """检查当前 M10P 驱动确实提供预期的唯一 /scan。"""
    lidar = _node(profile.data("lidar"), "lslidar_driver_node", profile.files["lidar"])
    _equal("雷达型号", lidar["lidar_model"], "M10P")
    _equal("雷达协议", lidar["lidar_type"], "X10")
    _equal("雷达坐标", lidar["frame_id"], "lidar_link")
    _equal("雷达扫描", lidar["laserscan_topic"], "/scan")
    _positive("雷达 min_range", lidar["min_range"])
    _positive("雷达 max_range", lidar["max_range"])
    if lidar["max_range"] <= lidar["min_range"]:
        raise ValueError("雷达 max_range 必须大于 min_range")
    if not lidar["serial_port"] or lidar["publish_scan"] is not True:
        raise ValueError("M10P 必须设置串口并发布 /scan")
    return lidar


def validate_monitor(profile: Profile) -> None:
    """诊断节点启动时也使用所选底盘的扫描、里程计和 frame。"""
    lidar = validate_lidar(profile)
    base = _node(profile.data("base"), "ekf_filter_node", profile.files["base"])
    monitor = _node(profile.data("monitor"), "kiwi_slam_monitor", profile.files["monitor"])
    _equal("诊断扫描", monitor["scan_topic"], lidar["laserscan_topic"])
    _equal("诊断里程计", monitor["odom_topic"], "/odometry/filtered")
    _equal("诊断雷达坐标", monitor["laser_frame"], lidar["frame_id"])
    _equal("诊断机体坐标", monitor["base_frame"], base["base_link_frame"])
    _equal("诊断里程计坐标", monitor["odom_frame"], base["odom_frame"])
    _equal("诊断地图坐标", monitor["map_frame"], base["map_frame"])


def validate_rf2o(profile: Profile) -> None:
    """履带只启动底盘 + EKF 时也检查激光里程计启动参数。"""
    if profile.odometry != "rf2o":
        return
    base = _node(profile.data("base"), profile.driver, profile.files["base"])
    ekf = _node(profile.data("base"), "ekf_filter_node", profile.files["base"])
    lidar = _node(profile.data("lidar"), "lslidar_driver_node", profile.files["lidar"])
    rf2o = _node(profile.data("rf2o"), "rf2o_laser_odometry", profile.files["rf2o"])
    adapter = _node(profile.data("rf2o"), "rf2o_covariance_adapter", profile.files["rf2o"])
    _equal("RF2O 扫描", rf2o["laser_scan_topic"], lidar["laserscan_topic"], "/scan")
    _equal("RF2O -> 协方差适配", rf2o["odom_topic"], adapter["source_topic"])
    _equal("协方差适配 -> EKF", adapter["output_topic"], ekf["odom0"])
    if adapter["source_topic"] == adapter["output_topic"]:
        raise ValueError("RF2O 协方差输入和输出不能相同")
    _equal("RF2O 机体", rf2o["base_frame_id"], ekf["base_link_frame"])
    _equal("RF2O 里程计", rf2o["odom_frame_id"], ekf["odom_frame"])
    _equal("协方差适配机体", adapter["base_frame"], ekf["base_link_frame"])
    _equal("协方差适配里程计", adapter["odom_frame"], ekf["odom_frame"])
    for key in (
        "pose_xy_variance", "pose_yaw_variance", "twist_vx_variance",
        "twist_vy_variance", "twist_wz_variance",
    ):
        _positive(f"RF2O {key}", adapter[key])
    if adapter["stationary_filter_enabled"] is not True:
        raise ValueError("履带 RF2O 必须启用静止零漂约束")
    _equal("RF2O 静止检测扫描", adapter["scan_topic"], lidar["laserscan_topic"])
    _equal("RF2O 静止检测 IMU", adapter["imu_topic"], base["imu_topic"])
    for key in (
        "stationary_window_s", "stationary_max_scan_median_delta_m",
        "stationary_max_translation_m", "stationary_max_yaw_rad",
        "stationary_max_gyro_rms_rad_s", "stationary_max_sensor_age_s",
    ):
        _positive(f"RF2O {key}", adapter[key])
    for key in ("stationary_min_scan_overlap_ratio", "stationary_min_valid_scan_ratio"):
        value = adapter[key]
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not 0.0 < value <= 1.0:
            raise ValueError(f"RF2O {key} 必须在 (0, 1] 范围内")
    _equal("IMU -> EKF", base["imu_topic"], ekf["imu0"])
    _equal("IMU 坐标", base["imu_frame_id"], "chassis_imu_link")
    acceleration = _node(profile.data("base"), "tracked_imu_acceleration", profile.files["base"])
    _equal("加速度原始输入", acceleration["source_topic"], base["imu_topic"])
    _equal("加速度原始坐标", acceleration["source_frame"], base["imu_frame_id"])
    _equal("加速度 -> EKF", acceleration["output_topic"], ekf["imu1"])
    _equal("加速度输出坐标", acceleration["output_frame"], ekf["base_link_frame"])
    if acceleration["output_topic"] == acceleration["source_topic"]:
        raise ValueError("校正加速度不能覆盖原始 IMU 话题")
    if ekf["imu1_config"] != [False] * 12 + [True, True, False]:
        raise ValueError("加速度 IMU 仅可融合 ax/ay，禁止重复融合 yaw 或姿态")
    if ekf["imu1_remove_gravitational_acceleration"] is not False:
        raise ValueError("加速度已经去重力，EKF 不得重复去重力")
    if not isinstance(acceleration["enabled"], bool):
        raise ValueError("加速度 enabled 必须为布尔值")
    model = _node(profile.data("base"), "tracked_robot_model", profile.files["base"])
    _equal("加速度杆臂", acceleration["imu_lever_xy"], model["imu_xyz"][:2])
    _equal("平面加速度模型旋转", model["imu_rpy"], [0.0, 0.0, 0.0])
    for key, length in (("accel_rotation", 9), ("rest_acceleration", 3),
                        ("acceleration_variance", 3), ("imu_lever_xy", 2)):
        values = acceleration[key]
        if not isinstance(values, list) or len(values) != length or any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                for v in values):
            raise ValueError(f"加速度 {key} 必须为 {length} 个有限数值")
    rotation = [acceleration["accel_rotation"][i:i + 3] for i in (0, 3, 6)]
    for i in range(3):
        for j in range(3):
            if abs(sum(a * b for a, b in zip(rotation[i], rotation[j])) - (i == j)) > 1e-5:
                raise ValueError("加速度旋转矩阵必须正交")
    cross = [rotation[0][1] * rotation[1][2] - rotation[0][2] * rotation[1][1],
             rotation[0][2] * rotation[1][0] - rotation[0][0] * rotation[1][2],
             rotation[0][0] * rotation[1][1] - rotation[0][1] * rotation[1][0]]
    if abs(sum(a * b for a, b in zip(cross, rotation[2])) - 1.0) > 1e-5:
        raise ValueError("加速度旋转矩阵不得镜像")
    gravity = [sum(a * b for a, b in zip(row, acceleration["rest_acceleration"])) for row in rotation]
    if math.hypot(*gravity[:2]) > 0.1 or not 8 < gravity[2] < 11:
        raise ValueError("加速度旋转矩阵与静止重力方向不一致")
    for variance in acceleration["acceleration_variance"]:
        _positive("加速度方差", variance)
    _equal("RF2O 实车原点初始化", rf2o["init_pose_from_topic"], "")
    if rf2o["publish_tf"] is not False:
        raise ValueError("履带 RF2O 必须关闭 TF，由 EKF 独占发布")

def validate_profile(profile: Profile, mode: str = "driver") -> bool:
    """按入口检查所需链路；建图不依赖导航参数，底盘联调不依赖建图。"""
    if mode not in ("driver", "mapping", "localization"):
        raise ValueError(f"未知运行模式：{mode}")
    base_yaml = profile.data("base")
    base = _node(base_yaml, profile.driver, profile.files["base"])
    ekf = _node(base_yaml, "ekf_filter_node", profile.files["base"])
    model_name = "kiwi_robot_model" if profile.chassis == "kiwi" else "tracked_robot_model"
    model = _node(base_yaml, model_name, profile.files["base"])
    teleop = _node(base_yaml, "kiwi_teleop_keyboard", profile.files["base"])
    _equal("/cmd_vel", base["cmd_vel_topic"], teleop["cmd_vel_topic"], "/cmd_vel")
    _positive("键盘 speed_m_s", teleop["speed_m_s"])
    _positive("键盘 yaw_rad_s", teleop["yaw_rad_s"])
    if teleop["speed_m_s"] > base["max_linear_speed_m_s"] or teleop["yaw_rad_s"] > base["max_angular_speed_rad_s"]:
        raise ValueError("键盘速度超过底盘物理限速")
    if profile.odometry == "wheel" and teleop["speed_m_s"] > base["max_lateral_speed_m_s"]:
        raise ValueError("键盘横移速度超过底盘物理限速")
    _equal("IMU -> EKF", base["imu_topic"], ekf["imu0"])
    _equal("机体坐标", ekf["base_link_frame"], "base_footprint")
    _equal("里程计坐标", ekf["odom_frame"], ekf["world_frame"], "odom")
    _equal("地图坐标", ekf["map_frame"], "map")
    if ekf["publish_tf"] is not True:
        raise ValueError("EKF 必须独占发布 odom -> base_footprint")
    if profile.odometry == "wheel":
        _equal("轮式里程计", base["wheel_odom_topic"], ekf["odom0"])
        motion = True
    else:
        _equal("履带横移上限", base.get("max_lateral_speed_m_s", 0.0), 0.0)
        _positive("track_separation_m", base["track_separation_m"])
        _positive("max_motor_command", base["max_motor_command"])
        calibrated = base["hardware_calibrated"] is True
        motion = base["enable_motion"] is True
        open_loop = base["open_loop_test_enabled"] is True
        trial = base["keyboard_trial_enabled"] is True
        if motion and (not calibrated or open_loop or trial):
            raise ValueError("正常运动必须先标定速度并关闭所有试车模式")
        if trial:
            if calibrated or open_loop:
                raise ValueError("临时键盘试车不能与已标定或原始电机试车模式并用")
            _positive("keyboard_trial_command_gain", base["keyboard_trial_command_gain"])
            _positive("keyboard_trial_limit_rad_s", base["keyboard_trial_limit_rad_s"])
            if base["keyboard_trial_limit_rad_s"] > base["max_motor_command"]:
                raise ValueError("临时键盘试车上限超过固件命令上限")
        if calibrated:
            _positive("command_per_m_s", base["command_per_m_s"])
    if mode == "driver":
        return motion

    lidar = validate_lidar(profile)
    if profile.odometry == "rf2o":
        validate_rf2o(profile)
        rf2o = _node(profile.data("rf2o"), "rf2o_laser_odometry", profile.files["rf2o"])
        if base["mapping_sensors_verified"] is not True:
            raise ValueError("履带传感器/外参及 RF2O 尚未核验；先设置 mapping_sensors_verified=true")

    if mode == "mapping":
        gate = _node(profile.data("gate"), "kiwi_mapping_scan_gate", profile.files["gate"])
        slam = _node(profile.data("slam"), "slam_toolbox", profile.files["slam"])
        _equal("建图机体", gate["base_frame"], slam["base_frame"], ekf["base_link_frame"])
        _equal("建图里程计", gate["odom_frame"], slam["odom_frame"], ekf["odom_frame"])
        _equal("建图地图", slam["map_frame"], ekf["map_frame"])
        _equal("建图扫描", gate["input_scan_topic"], lidar["laserscan_topic"])
        _equal("建图雷达坐标", gate["laser_frame"], lidar["frame_id"])
        _equal("SLAM 扫描", gate["output_scan_topic"], slam["scan_topic"], "/scan_slam")
        _equal("SLAM 最小量程", slam["min_laser_range"], lidar["min_range"])
        _equal("SLAM 最大量程", slam["max_laser_range"], lidar["max_range"])
        _equal("融合里程计", gate["odom_topic"], "/odometry/filtered")
        if "monitor" in profile.files:
            validate_monitor(profile)
        if profile.odometry == "rf2o":
            _equal("RF2O -> 门控", gate["source_odom_topic"], ekf["odom0"])
            _equal("IMU -> 门控", gate["imu_topic"], base["imu_topic"])
            if not gate["require_source_odom"] or not gate["require_imu_samples"]:
                raise ValueError("履带建图门控必须检查原始激光里程计和 IMU")
        return motion

    nav_yaml = profile.data("nav2")
    amcl = _node(nav_yaml, "amcl", profile.files["nav2"])
    controller = _node(nav_yaml, "controller_server", profile.files["nav2"])["FollowPath"]
    smoother = _node(nav_yaml, "velocity_smoother", profile.files["nav2"])
    _equal("AMCL 里程计", amcl["odom_frame_id"], ekf["odom_frame"])
    _equal("AMCL 地图", amcl["global_frame_id"], ekf["map_frame"])
    _equal("AMCL 扫描", amcl["scan_topic"], lidar["laserscan_topic"])
    _equal("AMCL 最小量程", amcl["laser_min_range"], lidar["min_range"])
    _equal("AMCL 最大量程", amcl["laser_max_range"], lidar["max_range"])
    _equal("AMCL 机体", amcl["base_frame_id"], ekf["base_link_frame"])
    _equal("Nav2 里程计", smoother["odom_topic"], "/odometry/filtered")
    bt = _node(nav_yaml, "bt_navigator", profile.files["nav2"])
    _equal("Nav2 地图", bt["global_frame"], ekf["map_frame"])
    _equal("Nav2 机体", bt["robot_base_frame"], ekf["base_link_frame"])
    _equal("Nav2 BT 里程计", bt["odom_topic"], "/odometry/filtered")
    for scope in ("local_costmap", "global_costmap"):
        costmap = nav_yaml[scope][scope]["ros__parameters"]
        _equal(f"{scope} 机体", costmap["robot_base_frame"], ekf["base_link_frame"])
        expected_frame = ekf["odom_frame"] if scope == "local_costmap" else ekf["map_frame"]
        _equal(f"{scope} 全局坐标", costmap["global_frame"], expected_frame)
        _equal(f"{scope} 扫描", costmap["obstacle_layer"]["scan"]["topic"], lidar["laserscan_topic"])
        if profile.odometry == "rf2o":
            needed = math.hypot(model["base_length_m"], model["base_width_m"]) / 2
            if costmap["robot_radius"] <= needed:
                raise ValueError(f"{scope}.robot_radius 未覆盖履带车体外形")
            if costmap["inflation_layer"]["inflation_radius"] <= costmap["robot_radius"]:
                raise ValueError(f"{scope}.inflation_radius 必须大于 robot_radius")
    if profile.odometry == "wheel":
        _equal("AMCL 全向模型", amcl["robot_model_type"], "nav2_amcl::OmniMotionModel")
        return True
    _equal("AMCL 差速模型", amcl["robot_model_type"], "nav2_amcl::DifferentialMotionModel")
    for key in ("min_vel_y", "max_vel_y", "acc_lim_y", "decel_lim_y"):
        _equal(f"履带控制器禁止横移 {key}", controller[key], 0.0)
    _equal("履带 vy_samples", controller["vy_samples"], 1)
    for key in ("max_velocity", "min_velocity", "max_accel", "max_decel"):
        _equal(f"履带速度平滑禁止横移 {key}", smoother[key][1], 0.0)
    _equal("履带速度平滑模式", smoother["feedback"], "OPEN_LOOP")
    _equal("履带导航速度 x", controller["max_vel_x"], smoother["max_velocity"][0])
    _equal("履带导航速度 yaw", controller["max_vel_theta"], smoother["max_velocity"][2])
    if smoother["velocity_timeout"] > base["cmd_vel_timeout_s"]:
        raise ValueError("履带 velocity_timeout 超过底盘命令看门狗")
    if controller["max_vel_x"] > base["max_linear_speed_m_s"] or controller["max_vel_theta"] > base["max_angular_speed_rad_s"]:
        raise ValueError("Nav2 速度上限超过履带底盘上限")
    if not calibrated or not motion or open_loop:
        raise ValueError("履带导航须关闭开环试车、标定速度换算并启用正常运动")
    return True
