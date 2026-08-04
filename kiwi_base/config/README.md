# Kiwi 参数入口

`kiwi_base.yaml` 现在同时包含节点运行参数、安全限值和三轮运动学参数，是正常启动时唯一需要加载的文件。节点声明中仍保留同样的默认值，因此即使省略参数文件也能启动；参数文件提供同名键时会覆盖节点默认值。

```bash
ros2 launch kiwi_base kiwi_base.launch.py
```

也可以从工作区根目录使用相对路径直接运行：

```bash
cd /home/ym/ros2_ws
ros2 run kiwi_base kiwi_base_node --ros-args \
  --params-file src/kiwi_base/config/kiwi_base.yaml
```

`kiwi_kinematics.yaml` 仅作为历史兼容文件保留，正常启动不再加载。launch 默认使用已安装包下的 `share/kiwi_base/config/kiwi_base.yaml`，因此不依赖当前工作目录；需要覆盖文件时可传 `params_file:=/path/to/your.yaml`。

运动学参数含义：

- `wheel_positions_m`：`[x1,y1,x2,y2,x3,y3]`，单位 m，坐标相对 `base_link`；
- `wheel_tangent_angles_deg`：三个轮子的驱动滚动切向角，单位度；
- `wheel_radius_m`：几何轮半径，单位 m；
- `wheel_sign`：编码器/电机正方向，取 `+1` 或 `-1`。
- `motor_reduction_ratio`：轮输出轴到电机轴的减速比；联调命令帧的三个 float 使用电机轴 `rad/s`，主机只在编码前乘一次该值。

当前初值是 `r=0.1015 m`、中心到轮心 `L=0.32256 m`、轮 1 在 `+y`。轮序、方向和有效接触半径必须用低速点动及外部距离测量重新确认。

电机/轮顺序固定为 ROS `wheel_joint_1/2/3` -> payload `motor_target_rad_s[0/1/2]` -> MCU CAN ID `1/2/3`。本版按现有 M3508 CAN 反馈和 PID 单位，将三个 float 定义为电机转子轴 `rad/s`；若下位机最终改成 `rpm` 或减速箱输出轴速度，只修改 codec 前后的单位转换，帧布局不变。

电机 ID、CAN 总线和 MCU 电流限制位于：
`/home/ym/ros2_ws/standard-robot-pro/application/typedef_robot/robot_param_kiwi.h`。
其中的减速比必须与本文件 `motor_reduction_ratio` 保持一致；两端分别存储是因为 ROS 与 Keil 工程不能直接共享头文件。

键盘联调必须由两个节点串联，键盘节点不直接占用串口：

```bash
# 终端 1：唯一串口所有者，完成 Twist -> 三轮逆解 -> 电机轴速度 -> 0x3A 控制帧。
source /opt/ros/humble/setup.bash
source /home/ym/ros2_ws/install/setup.bash
ros2 run kiwi_base kiwi_base_node --ros-args \
  --params-file /home/ym/ros2_ws/src/kiwi_base/config/kiwi_base.yaml

# 终端 2：发布 /cmd_vel_safe；w/s 前后、a/d 横移、q/e 旋转、空格或 x 停止。
source /opt/ros/humble/setup.bash
source /home/ym/ros2_ws/install/setup.bash
ros2 run kiwi_base kiwi_teleop_keyboard
```

首次联调应架空车轮并准备物理断电。键盘 250 ms 无新按键会归零，上位机命令和 MCU watchdog 均为 100 ms；这些软件保护不能代替动力回路急停。
