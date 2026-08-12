# kiwi_base 参数说明

本目录只有一个运行参数入口：`kiwi_base.yaml`。

```text
kiwi_base_node        底盘硬件、运动学、限幅、测量处理、话题和 frame
ekf_filter_node       轮式里程计 + C 板 IMU 的 robot_localization 参数
kiwi_teleop_keyboard  键盘速度与超时
kiwi_odom_test        滤波里程计调试话题
kiwi_robot_model      URDF 使用的底盘结构尺寸
```

固定运行接口：

```text
/cmd_vel                 底盘速度输入
/chassis/odom/raw        原始轮式里程计
/chassis/imu/data_raw    C 板 IMU
/chassis/feedback/raw    0x3B 解码调试数据
/joint_states            三个轮关节状态
/odometry/filtered       EKF 输出
```

传感器相对底盘的安装外参维护在 `kiwi_slam/urdf/kiwi.urdf.xacro`；导航避障半径维护在
`kiwi_navigation/config/nav2.yaml`。本目录不再保留按雷达型号拆分的 EKF 文件。
