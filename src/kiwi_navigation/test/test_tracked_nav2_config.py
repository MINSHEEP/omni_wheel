"""独立建图/导航入口读取同一活动底盘文件；履带无横移与编码器。"""
import importlib.util
import math
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from kiwi_platform import load_profile, validate_profile

PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE.parent


def load_launch(package, name):
    path = SOURCE / package / "launch" / name
    spec = importlib.util.spec_from_file_location(package + name.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context_for(module, **overrides):
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = "" if action.default_value is None else "".join(
                part.text if hasattr(part, "text") else str(part) for part in action.default_value
            )
    context.launch_configurations.update(overrides)
    return context


def test_tracked_navigation_has_its_own_complete_differential_yaml():
    nav = yaml.safe_load((PACKAGE / "config/tracked_nav2.yaml").read_text())
    base = yaml.safe_load((SOURCE / "kiwi_base/config/tracked_pwm.yaml").read_text())
    gate = yaml.safe_load((SOURCE / "kiwi_slam/config/tracked_mapping_gate.yaml").read_text())
    ekf = base["ekf_filter_node"]["ros__parameters"]
    motor = base["tracked_base_node"]["ros__parameters"]
    gate = gate["kiwi_mapping_scan_gate"]["ros__parameters"]
    assert nav["amcl"]["ros__parameters"]["robot_model_type"] == "nav2_amcl::DifferentialMotionModel"
    assert "alpha5" not in nav["amcl"]["ros__parameters"]
    assert ekf["odom0"] == gate["source_odom_topic"] == "/chassis/odom/laser"
    assert ekf["imu0"] == gate["imu_topic"] == "/chassis/imu/data_raw"
    assert gate["require_imu_samples"] and gate["require_source_odom"]
    controller = nav["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = nav["velocity_smoother"]["ros__parameters"]
    assert all(controller[key] == 0 for key in ("min_vel_y", "max_vel_y", "acc_lim_y", "decel_lim_y"))
    assert controller["vy_samples"] == 1
    assert all(smoother[key][1] == 0 for key in ("max_velocity", "min_velocity", "max_accel", "max_decel"))
    assert smoother["feedback"] == "OPEN_LOOP"
    assert controller["max_vel_x"] == smoother["max_velocity"][0] <= motor["max_linear_speed_m_s"]
    assert controller["max_vel_theta"] == smoother["max_velocity"][2] <= motor["max_angular_speed_rad_s"]
    model = base["tracked_robot_model"]["ros__parameters"]
    needed = math.hypot(model["base_length_m"], model["base_width_m"]) / 2
    for scope in ("local_costmap", "global_costmap"):
        costmap = nav[scope][scope]["ros__parameters"]
        assert costmap["robot_radius"] > needed
        assert costmap["inflation_layer"]["inflation_radius"] > costmap["robot_radius"]


def test_independent_mapping_selects_tracked_parameters_and_omits_nav2_until_calibrated(tmp_path):
    selector = tmp_path / "active.yaml"
    selector.write_text("chassis: tracked_pwm\n")
    original = load_profile(str(selector))
    base = original.data("base")
    base["tracked_base_node"]["ros__parameters"]["mapping_sensors_verified"] = True
    override = tmp_path / "verified.yaml"
    override.write_text(yaml.safe_dump(base))
    slam = load_launch("kiwi_slam", "kiwi_slam.launch.py")
    context = context_for(slam, platform_file=str(selector), base_params_file=str(override), use_rviz="false")
    actions = slam._setup(context)
    nodes = {action.node_executable: action for action in actions if hasattr(action, "node_executable")}
    assert {"tracked_base_node", "rf2o_laser_odometry_node", "ekf_node", "kiwi_mapping_scan_gate",
            "sync_slam_toolbox_node", "lslidar_driver_node"} <= nodes.keys()
    assert "kiwi_base_node" not in nodes
    for executable, key in (("tracked_base_node", "base"), ("ekf_node", "base"),
                            ("rf2o_laser_odometry_node", "rf2o"), ("kiwi_mapping_scan_gate", "gate"),
                            ("sync_slam_toolbox_node", "slam"), ("lslidar_driver_node", "lidar")):
        expected = str(override) if key == "base" else original.path(key)
        assert str(nodes[executable]._Node__parameters[0].evaluate(context)) == expected
    mapping = load_launch("kiwi_navigation", "mapping_navigation.launch.py")
    context = context_for(mapping, platform_file=str(selector), base_params_file=str(override), use_rviz="false")
    actions = mapping._setup(context)
    assert len(actions) == 1  # 未标定运动速度时只有 SLAM 链路
    args = dict(actions[0]._IncludeLaunchDescription__launch_arguments)
    assert args["base_params_file"] == str(override)
    assert args["platform_file"] == str(selector)
    with pytest.raises(ValueError, match="nav2_params_file 不可使用"):
        mapping._setup(context_for(mapping, platform_file=str(selector),
                                   base_params_file=str(override),
                                   nav2_params_file=original.path("nav2"), use_rviz="false"))
    navigation = load_launch("kiwi_navigation", "localization_navigation.launch.py")
    map_file = tmp_path / "map.yaml"
    map_file.write_text("image: map.pgm\nresolution: 0.05\norigin: [0,0,0]\n")
    (tmp_path / "map.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    with pytest.raises(ValueError, match="导航须关闭开环试车"):
        navigation._setup(context_for(navigation, platform_file=str(selector),
                                      base_params_file=str(override), map=str(map_file), use_rviz="false"))


def test_kiwi_mapping_keeps_nav2_and_omni_model(tmp_path):
    selector = tmp_path / "active.yaml"
    selector.write_text("chassis: kiwi\n")
    profile = load_profile(str(selector))
    assert validate_profile(profile, "mapping")
    assert profile.data("nav2")["amcl"]["ros__parameters"]["robot_model_type"] == "nav2_amcl::OmniMotionModel"
    mapping = load_launch("kiwi_navigation", "mapping_navigation.launch.py")
    actions = mapping._setup(context_for(mapping, platform_file=str(selector), use_rviz="false"))
    assert len(actions) == 2  # 建图 + Kiwi Nav2


def test_calibrated_tracked_localization_uses_own_nav2_and_rf2o(tmp_path):
    selector = tmp_path / "active.yaml"
    selector.write_text("chassis: tracked_pwm\n")
    profile = load_profile(str(selector))
    base = profile.data("base")
    base["tracked_base_node"]["ros__parameters"].update(
        mapping_sensors_verified=True, hardware_calibrated=True,
        enable_motion=True, open_loop_test_enabled=False, command_per_m_s=10.0,
    )
    calibrated = tmp_path / "calibrated.yaml"
    calibrated.write_text(yaml.safe_dump(base))
    map_file = tmp_path / "map.yaml"
    map_file.write_text("image: map.pgm\nresolution: 0.05\norigin: [0,0,0]\n")
    (tmp_path / "map.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    launch = load_launch("kiwi_navigation", "localization_navigation.launch.py")
    context = context_for(launch, platform_file=str(selector), base_params_file=str(calibrated),
                          map=str(map_file), use_rviz="false")
    actions = launch._setup(context)
    nodes = [action.node_executable for action in actions if hasattr(action, "node_executable")]
    assert nodes == ["robot_state_publisher", "tracked_base_node", "lslidar_driver_node",
                     "rf2o_laser_odometry_node", "ekf_node"]
    includes = [action for action in actions if action.__class__.__name__ == "IncludeLaunchDescription"]
    assert len(includes) == 2
    localization = dict(includes[0]._IncludeLaunchDescription__launch_arguments)
    navigation = dict(includes[1]._IncludeLaunchDescription__launch_arguments)
    assert localization["params_file"] == profile.path("nav2")
    assert localization["map"] == str(map_file)
    rewritten = navigation["params_file"]
    parameters = yaml.safe_load(Path(rewritten.perform(context)).read_text())
    assert parameters["amcl"]["ros__parameters"]["robot_model_type"] == (
        "nav2_amcl::DifferentialMotionModel"
    )
    assert parameters["velocity_smoother"]["ros__parameters"]["max_velocity"][1] == 0.0


def test_localization_rejects_missing_map_image_before_starting_nodes(tmp_path):
    launch = load_launch("kiwi_navigation", "localization_navigation.launch.py")
    map_file = tmp_path / "map.yaml"
    map_file.write_text("image: missing.pgm\nresolution: 0.05\norigin: [0,0,0]\n")
    with pytest.raises(ValueError, match="地图图像文件不存在"):
        launch._setup(context_for(launch, map=str(map_file), use_rviz="false"))
