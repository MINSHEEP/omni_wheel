# kiwi_navigation

建图、定位导航是两个独立入口，都读取 `kiwi_platform/config/active_platform.yaml`。Kiwi 使用 `config/nav2.yaml`（AMCL 全向模型）；两电机履带使用 `config/tracked_nav2.yaml`（AMCL 差速模型、零横移、RF2O + IMU、0.48 m 车体半径）。两个文件均为完整 Nav2 参数，不在启动时从底盘 YAML 动态拼接。底盘、门控、SLAM 和 URDF 由同一活动档案选取。

```bash
cd "$HOME/ros2_ws"
source tools/activate_workspace.sh
ros2 launch kiwi_navigation mapping_navigation.launch.py use_rviz:=true
# 地图保存后停止建图，再定位导航：
ros2 launch kiwi_navigation localization_navigation.launch.py \
  map:="$HOME/ros2_ws/maps/warehouse.yaml" use_rviz:=true
```

Kiwi 建图入口同时启动 Nav2；履带在速度未标定时只启动建图链路。履带需先完成雷达、IMU 和 RF2O 核验并设置 `mapping_sensors_verified=true`；导航还需要速度换算标定并启用运动。定位导航会在启动任何硬件节点前检查地图文件、完整参数链和关锁。

Nav2 速度源统一送 `/cmd_vel`；Web 使用 `navigation_core.launch.py`，通过 `/kiwi_web/nav_raw` 和 `/kiwi_web/nav_cmd_vel` 接入网页速度仲裁，同样读取活动底盘档案。导航期间不要同时运行键盘。平台文件与全部参数映射、待标定项见 [平台架构](../../documents/platform_architecture.md)。
