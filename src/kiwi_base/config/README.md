# kiwi_base 参数说明

统一平台按底盘选择一份专属参数：Kiwi 使用 `kiwi_base.yaml`，履带使用 `tracked_pwm.yaml`。
两份 YAML 各自包含底盘、EKF、模型、键盘和建图门控参数；履带 YAML 还包含差速 AMCL、无横移控制器、速度平滑和避障外形等 Nav2 覆盖值。

以下是原 Kiwi 节点和话题；履带改用 `tracked_base_node`，不会发布轮速里程计或 `/joint_states`。

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

Kiwi 的安装外参维护在 `kiwi_slam/urdf/kiwi.urdf.xacro`，履带占位外参维护在
`tracked_pwm.yaml` 并由 `kiwi_slam/urdf/tracked.urdf.xacro` 使用。公共 Nav2 参数在
`kiwi_navigation/config/nav2.yaml`；履带专属避障半径在 `tracked_pwm.yaml`。

履带的 `hardware_calibrated=false`、`enable_motion=false` 保持普通 `/cmd_vel` 锁定；
`open_loop_test_enabled=true` 时在统一入口 `mode:=driver`，或传感器已核验后的
`mode:=mapping` 手动建图中，用 `/chassis/open_loop_motor_cmd` 试发 `[左, 右]` 两路原始 rad/s 命令。
试车上限、超时和后续标定步骤见工作区根 [README](../../../README.md)。
