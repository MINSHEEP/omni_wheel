"""检查镭神 M10P 的 Nav2 参数和启动模式。"""

import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def load_yaml(name: str):
    return yaml.safe_load((PACKAGE / "config" / name).read_text(encoding="utf-8"))


def test_nav2_uses_omni_frames_with_lateral_motion_disabled():
    """现场要求「转向+直行」：横移暂禁用；恢复全向时同步改回本断言。"""
    config = load_yaml("nav2.yaml")
    amcl = config["amcl"]["ros__parameters"]
    controller = config["controller_server"]["ros__parameters"]
    dwb = controller["FollowPath"]
    assert amcl["robot_model_type"] == "nav2_amcl::OmniMotionModel"
    assert amcl["base_frame_id"] == "base_footprint"
    assert controller["odom_topic"] == "/odometry/filtered"
    assert dwb["min_vel_y"] == 0.0 == dwb["max_vel_y"]
    assert dwb["vy_samples"] == 1
    assert config["global_costmap"]["global_costmap"]["ros__parameters"]["global_frame"] == "map"
    assert config["local_costmap"]["local_costmap"]["ros__parameters"]["global_frame"] == "odom"


def test_controller_rotates_first_and_reverse_is_disabled():
    """后方无传感器：锁定「先转向后平移 + 禁止后退 + 无 backup 恢复」的安全合约。"""
    config = load_yaml("nav2.yaml")
    follow = config["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow["plugin"] == "nav2_rotation_shim_controller::RotationShimController"
    assert follow["primary_controller"] == "dwb_core::DWBLocalPlanner"
    # 约 20°以内交给 DWB 连续跟踪，避免定位和路径切线小抖动反复触发原地校向。
    assert follow["angular_dist_threshold"] == 0.35
    assert follow["min_vel_x"] == 0.0
    behaviors = config["behavior_server"]["ros__parameters"]["behavior_plugins"]
    assert "backup" not in behaviors
    # 移除 backup 插件后，默认行为树会因等不到 backup server 而使 bt_navigator
    # 激活失败；必须换用无 BackUp 的自定义行为树，且由两个 launch 注入路径。
    bt = config["bt_navigator"]["ros__parameters"]
    assert "default_nav_to_pose_bt_xml" in bt
    assert "default_nav_through_poses_bt_xml" in bt
    for name in (
        "navigate_to_pose_no_backup.xml",
        "navigate_through_poses_no_backup.xml",
    ):
        tree = (PACKAGE / "behavior_trees" / name).read_text(encoding="utf-8")
        assert "<BackUp" not in tree
        assert "<Spin" in tree and "ClearEntireCostmap" in tree
    shared_launch = (PACKAGE.parent / "kiwi_platform" / "kiwi_platform" / "launching.py").read_text(encoding="utf-8")
    assert "RewrittenYaml" in shared_launch
    assert "navigate_to_pose_no_backup.xml" in shared_launch
    assert "navigate_through_poses_no_backup.xml" in shared_launch
    mapping_launch = (PACKAGE / "launch/mapping_navigation.launch.py").read_text(encoding="utf-8")
    localization_launch = (PACKAGE / "launch/localization_navigation.launch.py").read_text(encoding="utf-8")
    assert "nav2_parameters(nav_profile)" in mapping_launch
    assert "nav2_parameters(profile)" in localization_launch
    # 膨胀半径必须覆盖必撞区（大于 robot_radius），否则规划器会贴墙。
    # 驱动已区分前向无回波 Inf 和后方盲区 NaN，因此 Inf 必须参与动态清除。
    for scope in ("local_costmap", "global_costmap"):
        costmap = config[scope][scope]["ros__parameters"]
        assert costmap["robot_radius"] == 0.37
        assert costmap["inflation_layer"]["inflation_radius"] > costmap["robot_radius"]
        assert costmap["inflation_layer"]["inflation_radius"] == 0.85
        assert costmap["inflation_layer"]["cost_scaling_factor"] == 2.0
        assert costmap["obstacle_layer"]["scan"]["inf_is_valid"] is True
        # 大地图只发增量，避免 DDS/RViz 每周期重传并重绘整张 costmap。
        assert costmap["always_send_full_costmap"] is False

    # 定位导航基于已加载的静态地图，不让未知栅格参与通行代价。
    global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
    assert global_costmap["track_unknown_space"] is False

    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    assert planner["plugin"] == "nav2_theta_star_planner/ThetaStarPlanner"
    assert planner["how_many_corners"] == 8
    assert planner["w_euc_cost"] == 1.0
    assert planner["w_traversal_cost"] == 3.0
    # Theta* 不支持 SmacPlanner2D 的这些参数，残留参数会掩盖配置错误。
    for obsolete in ("tolerance", "max_iterations", "max_planning_time", "smooth_path"):
        assert obsolete not in planner

    smoother = config["smoother_server"]["ros__parameters"]
    assert smoother["smoother_plugins"] == ["constrained_smoother"]
    constrained = smoother["constrained_smoother"]
    assert constrained["plugin"] == "nav2_constrained_smoother/ConstrainedSmoother"
    assert constrained["reversing_enabled"] is False
    assert constrained["minimum_turning_radius"] == 0.40
    assert constrained["path_downsampling_factor"] == 2
    assert constrained["w_cost"] > 0.0

    # 规划后必须调用带碰撞检查的限时平滑；失败由 ForceSuccess 快速回退
    # 黑板中的 Theta* 原始路径，不能触发清图、旋转或等待恢复。
    for name in (
        "navigate_to_pose_no_backup.xml",
        "navigate_through_poses_no_backup.xml",
    ):
        root = ET.parse(PACKAGE / "behavior_trees" / name).getroot()
        smooth = root.find(".//ForceSuccess/SmoothPath")
        assert smooth is not None
        assert smooth.attrib["unsmoothed_path"] == "{path}"
        assert smooth.attrib["smoothed_path"] == "{path}"
        assert smooth.attrib["smoother_id"] == "constrained_smoother"
        assert float(smooth.attrib["max_smoothing_duration"]) <= 0.20
        assert smooth.attrib["check_for_collisions"] == "true"

        # 路径有效时必须复用当前路径，禁止固定频率重复规划打断 RotationShim。
        compute_if_needed = root.find(".//Fallback[@name='ComputePathIfNeeded']")
        assert compute_if_needed is not None
        reuse = compute_if_needed.find("./ReactiveSequence[@name='ReuseValidPath']")
        assert reuse is not None
        assert reuse.find("./Inverter/GlobalUpdatedGoal") is not None
        assert reuse.find("./IsPathValid").attrib["path"] == "{path}"

    progress = config["controller_server"]["ros__parameters"]["progress_checker"]
    rotation_shim = config["controller_server"]["ros__parameters"]["FollowPath"]
    behavior = config["behavior_server"]["ros__parameters"]
    assert progress["plugin"] == "nav2_controller::PoseProgressChecker"
    assert progress["required_movement_angle"] == 0.15
    assert progress["movement_time_allowance"] == 10.0
    assert rotation_shim["angular_disengage_threshold"] < rotation_shim["angular_dist_threshold"]
    # 实测闭环模式会因静止角速度反馈持续为零，把校向命令锁死在 0.03 rad/s；
    # 此开环仅用于 RotationShim 加速度斜坡，不改变反馈里程计积分来源。
    assert rotation_shim["closed_loop"] is False
    assert behavior["rotational_acc_lim"] == 0.60


def test_velocity_smoother_is_the_direct_cmd_vel_output():
    config = load_yaml("nav2.yaml")
    smoother = config["velocity_smoother"]["ros__parameters"]
    # 平滑频率与底盘 50 Hz 控制循环一致，在两次 DWB 输出之间生成更细速度台阶。
    assert smoother["smoothing_frequency"] == 50.0
    # y 分量置零禁止横移、x 下限置零禁止后退，与 DWB 限速一致。
    assert smoother["max_velocity"] == [0.30, 0.0, 0.80]
    assert smoother["min_velocity"] == [0.0, 0.0, -0.80]
    assert smoother["velocity_timeout"] == 0.50
    assert not (PACKAGE / "config" / "cmd_relay.yaml").exists()


def test_launch_modes_share_selected_lidar_without_adapter_or_relay():
    mapping = (PACKAGE / "launch" / "mapping_navigation.launch.py").read_text(encoding="utf-8")
    localization = (PACKAGE / "launch" / "localization_navigation.launch.py").read_text(encoding="utf-8")
    shared = (PACKAGE.parent / "kiwi_platform" / "kiwi_platform" / "launching.py").read_text(encoding="utf-8")
    profile = (PACKAGE.parent / "kiwi_platform" / "config" / "profiles" / "kiwi.yaml").read_text(encoding="utf-8")
    assert "kiwi_slam.launch.py" in mapping
    assert "validate_profile(profile, \"mapping\")" in mapping
    assert "localization_launch.py" in localization
    assert "robot_actions(profile)" in localization
    assert "lslidar_driver_node" in shared
    assert "kiwi_slam/config/ls_m10p_uart.yaml" in profile
    for launch in (mapping, localization):
        assert "kiwi_nav_cmd_relay" not in launch


def test_navigation_rviz_uses_map_frame():
    rviz = (PACKAGE / "config" / "navigation.rviz").read_text(encoding="utf-8")
    assert "Fixed Frame: map" in rviz
    assert "Name: Smoothed Global Plan" in rviz
    assert "Value: /plan_smoothed" in rviz
    assert "Name: Raw Global Plan" in rviz
    assert "camera_init" not in rviz


def test_navigation_declares_active_planning_plugins():
    """运行依赖必须与配置中的实际规划和平滑插件一致。"""
    root = ET.parse(PACKAGE / "package.xml").getroot()
    dependencies = {element.text for element in root.findall("exec_depend")}
    assert "nav2_theta_star_planner" in dependencies
    assert "nav2_constrained_smoother" in dependencies
    assert "nav2_smac_planner" not in dependencies
