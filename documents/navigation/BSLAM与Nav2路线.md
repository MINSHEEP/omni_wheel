# B 路线：SLAM、地图定位与 Nav2

> 责任目标：消费 A 的标准里程计和 TF，交付地图、`map→odom`、全向导航和安全后的
> `/cmd_vel_safe`。
> 对接契约：[接口契约.md](接口契约.md)

## 1. 完成定义

B 负责机器人“在哪里、去哪里、如何避障”，不处理电机轮序、减速比、USB 帧或 IMU
安装方向。

```text
输入：/scan、/odometry/filtered、odom→base_footprint、机器人静态 TF
输出：地图、map→odom、路径、/cmd_vel_nav、/cmd_vel_safe、/software_estop
```

## 2. 建议代码边界

```text
新增 src/kiwi_slam/
  config/slam_mapping.yaml
  launch/mapping.launch.py
  rviz/mapping.rviz

新增 src/kiwi_navigation/
  config/amcl.yaml
  config/nav2.yaml
  config/collision_monitor.yaml
  config/velocity_smoother.yaml
  config/cmd_mux.yaml
  launch/localization.launch.py
  launch/navigation.launch.py
  maps/
  behavior_trees/
  rviz/navigation.rviz
  test/
```

B 不修改 `kiwi_base` 的轮序/协议来适配 Nav2。若 `/cmd_vel_safe` 的标准方向在实车上错误，
由 A 修复硬件适配层。

## 3. B0：脱离实车建立可回放基线

B 可以在 A 完成 G1 前并行开发：

1. 使用 A 提供的 rosbag，或创建明确标记为测试用的 `odom→base_footprint` 替身。
2. 回放时统一 `use_sim_time=true`，硬件串口节点默认禁用。
3. 先验证 TF、scan 和 odom 接口，再调算法参数。
4. 把实车依赖限制在 launch 参数，不把设备路径写进 SLAM/Nav2 配置。

回放门禁：`map→odom→base_footprint→lidar_link` 在整个 bag 时间范围内都可查询，
`/scan` 不出现大段时间倒退或 frame 错误。

## 4. B1：SLAM Toolbox 建图

建图模式只启动 SLAM Toolbox，不启动 AMCL。

参数起点：

```yaml
slam_toolbox:
  ros__parameters:
    mode: mapping
    map_frame: map
    odom_frame: odom
    base_frame: base_footprint
    scan_topic: /scan
    use_scan_matching: true
    resolution: 0.05
    transform_publish_period: 0.02
    map_update_interval: 2.0
```

调参顺序：

1. 先确认 A 的 scan、TF、里程计方向和时间戳。
2. 低速走小闭环，检查墙体厚度和重影。
3. 再调最小位移/转角、匹配搜索范围和回环阈值。
4. 最后走完整环境并保存地图，不用回环参数掩盖里程计轴错误。

建图交付：

- 地图 `.yaml + .pgm`，文件名包含场地和版本。
- 建图参数、rosbag、RViz 截图和闭环轨迹。
- 记录地图分辨率、原点、禁行区和实际测量基准。

验收：地图无持续双墙/扇形重影，闭环后全局结构没有明显撕裂，重新加载尺寸一致。

## 5. B2：已知地图定位

第一阶段使用 `map_server + AMCL`，关闭 SLAM Toolbox。三轮全向底盘必须使用 Omni
运动模型，并统一 frame/topic：

```yaml
amcl:
  ros__parameters:
    robot_model_type: nav2_amcl::OmniMotionModel
    global_frame_id: map
    odom_frame_id: odom
    base_frame_id: base_footprint
    scan_topic: /scan
```

定位验收：

1. 保存地图后重启系统，设置初始位姿可以收敛。
2. 横移时粒子云和机器人方向不会按差速模型错误发散。
3. 原地转动和绕矩形后能回到地图相同位置。
4. AMCL 是定位模式下 `map→odom` 的唯一发布者。
5. 定位质量恶化时触发诊断/停止策略，不继续盲发速度。

SLAM Toolbox localization 模式可作为后续对比，但第一版不要与 AMCL 同时发布 TF。

## 6. B3：Nav2 全向基线

### 6.1 Controller

ROS 2 Humble 第一版使用 DWB，必须显式启用横向采样：

```text
min_vel_y < 0
max_vel_y > 0
vy_samples > 1
acc_lim_y > 0
decel_lim_y < 0
```

同时冻结：

```text
odom_topic=/odometry/filtered
global_frame=map
robot_base_frame=base_footprint
vx/vy/wz 最大值不得超过 A 验证过的底盘安全能力
```

先用保守速度完成纯前进、纯横移、斜向和原地转动，再调 critic。若 DWB 不能稳定满足
性能要求，第二阶段再评估 MPPI Omni；不能在第一阶段同时更换定位、控制器和速度链。

### 6.2 Planner 和 costmap

- 全局规划第一版使用 SmacPlanner2D 或项目已验证的二维规划器。
- footprint 使用实测外轮廓加安全余量，不用随意半径替代非圆形机体。
- 全局/局部 costmap 的 `obstacle_layer`、`inflation_layer` 使用 `/scan`。
- local costmap 使用 `odom`，global costmap 使用 `map`。
- 雷达最大有效范围、raytrace 范围和 inflation 半径必须与实测一致。

### 6.3 行为树与失败处理

至少覆盖：

- 正常单点导航、取消目标、规划超时。
- 控制器失败、路径被阻挡和有限次数恢复。
- 定位不可用或 TF 超时时停止输出。
- 急停期间拒绝恢复运动；解除后需要新命令或重新下发目标。

## 7. B4：速度仲裁和碰撞保护

冻结链路：

```text
/cmd_vel_nav + /cmd_vel_keyboard
  -> cmd mux
  -> /cmd_vel_selected
  -> nav2_velocity_smoother
  -> /cmd_vel_smoothed
  -> nav2_collision_monitor
  -> /cmd_vel_safe
```

优先级：软件急停 > 键盘 > Nav2。当前 `twist_mux` 未安装，B 可以安装 Humble 对应包，
也可以实现小型仲裁节点，但必须具有：

1. 每个输入源独立超时。
2. 高优先级接管时立即覆盖低优先级。
3. 所有输入失效时持续输出零速或显式无命令状态。
4. 只处理 REP-103 Twist，不承担硬件换轴。
5. `/software_estop` 以 10 Hz 心跳发布，超过 300 ms 无心跳按急停处理。

`velocity_smoother` 负责速度/加速度约束，`collision_monitor` 负责基于传感器的减速或停止。
A 的 100 ms 超时、MCU watchdog 和物理急停仍然必须存在。

## 8. B5：三种独立启动模式

### 8.1 建图

```text
A 平台 launch
  + slam_toolbox mapping
  + RViz mapping
  + 键盘安全链
```

输出 `/map` 和 `map→odom`，不启动 AMCL/Nav2 controller。

### 8.2 地图定位

```text
A 平台 launch
  + map_server
  + AMCL
  + RViz localization
```

用于初始位姿、重定位和定位质量验收，不要求发送导航目标。

### 8.3 正式导航

```text
A 平台 launch
  + map_server + AMCL
  + Nav2 planner/controller/BT
  + cmd mux + velocity_smoother + collision_monitor
```

启动文件必须保证失败时不会同时拉起 SLAM 和 AMCL。

## 9. B 的测试矩阵

```text
类别          用例                                       通过条件
配置测试      YAML 可解析、插件和 frame/topic 存在        launch 前失败可见
TF 测试       map→odom 唯一、完整 TF 链                   无竞争/断链
SLAM 回放     小闭环、完整场地、回环                      地图无明显重影
定位回放      初始位姿、绑架恢复、横移                    Omni 模型稳定
控制测试      +x、+y、斜向、+wz、窄通道                  输出包含合理 vy
仲裁测试      Nav2→键盘接管→释放                          优先级和超时正确
碰撞测试      前方/侧方临时障碍                           减速、绕行或停止
失败测试      scan/odom/TF 丢失、规划失败、定位丢失       /cmd_vel_safe 归零
实车回归      多次相同路线                                成功率和误差有记录
```

建议验证命令：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select kiwi_slam kiwi_navigation
colcon test --packages-select kiwi_slam kiwi_navigation \
  --event-handlers console_direct+
colcon test-result --verbose
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic hz /cmd_vel_nav /cmd_vel_safe
```

## 10. B 交付给 A 的接口包

```text
commit.txt                  根仓库 commit
parameters/                slam、AMCL、Nav2、安全链 YAML
maps/                      地图 yaml/pgm 和版本说明
bags/navigation_<date>/    导航与障碍测试 rosbag
reports/map.md             建图闭环和重影结论
reports/localization.md    重定位和横移结论
reports/safety.md          接管、急停、障碍、失败归零结果
```

同时明确告诉 A：允许的最大 `vx/vy/wz`、加减速度、命令频率和超时时间。A 的底盘限幅可以
更严格，但不能比 B 假设的更宽松后又把安全责任推回 Nav2。

## 11. B 不负责的内容

- 不修改 C 板协议、电机 rpm/rad/s 换算、轮序或减速比。
- 不发布 `odom→base_footprint`，也不从激光匹配伪造轮里程计。
- 不在 Nav2 参数中补偿雷达 `frame_id=map`、IMU 轴错误或底盘换轴错误。
- 不用加大 inflation 或降低速度掩盖 TF/时间戳问题。
- 不把软件碰撞保护描述成硬件安全急停。
