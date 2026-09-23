# 底盘参数目录

`kiwi_base` 仍是实际底盘驱动包：`kiwi_base_node` 控制三轮全向底盘，`tracked_base_node` 控制两电机履带。`kiwi_platform` 只负责读取活动选择文件并选取参数，自己不连接下位机，也不替代这两个节点。

| 本目录文件 | 使用条件 | 主要内容 |
| --- | --- | --- |
| `kiwi_base.yaml` | `active_platform.yaml` 中 `chassis: kiwi` | Kiwi 驱动、轮式里程计 EKF、机身模型和共用键盘参数 |
| `tracked_pwm.yaml` | `active_platform.yaml` 中 `chassis: tracked_pwm` | 履带驱动、RF2O + IMU EKF、机身模型和共用键盘参数 |

配置读取关系是：`kiwi_platform/config/active_platform.yaml` 选择底盘 → `kiwi_platform/config/profiles/<底盘>.yaml` 指定各功能包文件 → 本目录对应的底盘 YAML 交给 `kiwi_base` 驱动、EKF、键盘和 URDF。两份底盘 YAML 都在运行链中，分别服务不同硬件。

建图门控与 SLAM 参数在 `kiwi_slam/config/`，Nav2 参数在 `kiwi_navigation/config/`，由同一底盘档案同步切换。完整文件表、启动命令与标定关锁见[平台架构](../../../documents/platform_architecture.md)和[工作区 README](../../../README.md)。
