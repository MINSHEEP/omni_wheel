"""检查镭神 M10P 的 Nav2 参数和启动模式。"""

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
    for launch_name in (
        "mapping_navigation.launch.py",
        "localization_navigation.launch.py",
    ):
        launch = (PACKAGE / "launch" / launch_name).read_text(encoding="utf-8")
        assert "RewrittenYaml" in launch
        assert "navigate_to_pose_no_backup.xml" in launch
        assert "navigate_through_poses_no_backup.xml" in launch
    # 膨胀半径必须覆盖必撞区（大于 robot_radius），否则规划器会贴墙。
    # M10P 的 inf 同时表示无回波和屏蔽扇区，不得用来清除未观测空间。
    for scope in ("local_costmap", "global_costmap"):
        costmap = config[scope][scope]["ros__parameters"]
        assert costmap["robot_radius"] == 0.37
        assert costmap["inflation_layer"]["inflation_radius"] > costmap["robot_radius"]
        assert costmap["inflation_layer"]["inflation_radius"] == 0.85
        assert costmap["inflation_layer"]["cost_scaling_factor"] == 2.0
        assert costmap["obstacle_layer"]["scan"]["inf_is_valid"] is False
        # 大地图只发增量，避免 DDS/RViz 每周期重传并重绘整张 costmap。
        assert costmap["always_send_full_costmap"] is False

    progress = config["controller_server"]["ros__parameters"]["progress_checker"]
    behavior = config["behavior_server"]["ros__parameters"]
    assert progress["movement_time_allowance"] == 15.0
    assert behavior["rotational_acc_lim"] == 0.60


def test_velocity_smoother_is_the_direct_cmd_vel_output():
    config = load_yaml("nav2.yaml")
    smoother = config["velocity_smoother"]["ros__parameters"]
    # y 分量置零禁止横移、x 下限置零禁止后退，与 DWB 限速一致。
    assert smoother["max_velocity"] == [0.30, 0.0, 0.80]
    assert smoother["min_velocity"] == [0.0, 0.0, -0.80]
    assert smoother["velocity_timeout"] == 0.50
    assert not (PACKAGE / "config" / "cmd_relay.yaml").exists()


def test_launch_modes_use_m10p_without_adapter_or_relay():
    mapping = (PACKAGE / "launch" / "mapping_navigation.launch.py").read_text(
        encoding="utf-8"
    )
    localization = (PACKAGE / "launch" / "localization_navigation.launch.py").read_text(
        encoding="utf-8"
    )
    assert "condition=IfCondition(start_slam)" in mapping
    assert "slam_toolbox" not in localization
    assert "localization_launch.py" in localization
    for launch in (mapping, localization):
        # 雷达统一复用 kiwi_slam 的 M10P 封装，保证运行时 /scan 只有一个发布者。
        assert '"m10p_uart_lidar.launch.py"' in launch
        assert '"ls_m10p_uart.yaml"' in launch
        assert "kiwi_nav_cmd_relay" not in launch


def test_navigation_rviz_uses_map_frame():
    rviz = (PACKAGE / "config" / "navigation.rviz").read_text(encoding="utf-8")
    assert "Fixed Frame: map" in rviz
    assert "camera_init" not in rviz
