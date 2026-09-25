# Kiwi ROS 2 操作手册

## 1. 环境准备与底盘切换

每个新终端启动前必须加载环境：

```bash
cd ~/ros2_ws
# 编译工作空间
colcon build --symlink-install 
source tools/activate_workspace.sh

```

切换活动底盘（修改后重启相关节点生效）：

```bash
# 切换为履带底盘
sed -i 's/^chassis:.*/chassis: tracked_pwm/' src/kiwi_platform/config/active_platform.yaml

# 切换为 Kiwi 底盘
sed -i 's/^chassis:.*/chassis: kiwi/' src/kiwi_platform/config/active_platform.yaml

```

---

## 2. 基础测试（底盘与 EKF）

```bash
# 启动底盘、传感器与 EKF 融合
ros2 launch kiwi_base kiwi_base_ekf.launch.py

# 启动键盘遥控（W/S 前后，Q/E 转向，空格/X 刹车）
ros2 run kiwi_base kiwi_teleop_keyboard

```

---

## 3. 移动建图流程

### 步骤 1：启动建图节点

```bash
# 方式 A：纯建图（推荐履带底盘使用）
ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true

# 方式 B：建图 + Nav2（支持探索导航）
ros2 launch kiwi_navigation mapping_navigation.launch.py use_rviz:=true

```

### 步骤 2：遥控建图

另起终端运行键盘控制车辆移动建图：

```bash
cd ~/ros2_ws && source tools/activate_workspace.sh
ros2 run kiwi_base kiwi_teleop_keyboard

```

> **可选（履带建图录包）：**
> ```bash
> mkdir -p ~/ros2_ws/bags
> ros2 bag record -o ~/ros2_ws/bags/tracked_$(date +%Y%m%d_%H%M%S) \
>   /scan /scan_slam /chassis/imu/data_raw /chassis/odom/laser \
>   /odometry/filtered /tf /tf_static /map
> 
> ```
> 
> 

### 步骤 3：保存地图

建图完成后，保存地图文件至 `~/ros2_ws/maps/`：

```bash
mkdir -p ~/ros2_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/warehouse

```

---

## 4. 已知地图定位与导航

启动定位与 Nav2 导航（必须指定 `.yaml` 地图路径）：

```bash
ros2 launch kiwi_navigation localization_navigation.launch.py \
  map:=$HOME/ros2_ws/maps/warehouse.yaml \
  use_rviz:=true

```

> RViz 启动后：
> 1. 点击工具栏 **2D Pose Estimate** 标记车辆初始位姿。
> 2. 点击 **Nav2 Goal** 发送目标点。
> 
> 

---

## 5. Web 控制台（可选）

```bash
ros2 launch kiwi_web web.launch.py

```

---

## 6. 快捷排错命令

```bash
# 检查关键数据流频率
ros2 topic hz /scan
ros2 topic hz /odometry/filtered
ros2 topic hz /map

# 检查坐标变换链
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint lidar_link

```