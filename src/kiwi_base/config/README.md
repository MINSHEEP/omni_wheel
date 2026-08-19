# kiwi_base 参数说明

本目录只有一个运行参数入口：`kiwi_base.yaml`。

```text
kiwi_base_node        底盘硬件、运动学、限幅、反馈里程计、话题和 frame
ekf_filter_node       电机反馈里程计的 robot_localization 参数
kiwi_teleop_keyboard  键盘速度与超时
kiwi_odom_test        滤波里程计调试话题
kiwi_robot_model      URDF 使用的底盘结构尺寸
```

固定运行接口：

```text
/cmd_vel                 底盘速度输入
/chassis/odom/raw        滤波后的电机实测速度推算的里程计
/chassis/imu/data_raw    异步 0x3B IMU 遥测
/chassis/feedback/raw    异步 0x3B 原始字段调试消息
/joint_states            电机实测速度换算的三个轮关节状态
/odometry/filtered       EKF 输出
```

`0x3A` 命令没有 ACK，节点不等待命令应答；同一串口上的 `0x3B` 遥测按周期非阻塞读取，
断流不会阻止 TX。电机实测值经低通、死区和减速比换算后驱动里程计；反馈断流期间不使用
`/cmd_vel` 补算位移，恢复首帧也不会跨越未知时间积分。

传感器相对底盘的安装外参维护在 `kiwi_slam/urdf/kiwi.urdf.xacro`；导航避障半径维护在
`kiwi_navigation/config/nav2.yaml`。本目录不再保留按雷达型号拆分的 EKF 文件。
