# MID360S 第三方源码与构建边界

本工作区直接收录固定版本的源码，首次检出不需要再克隆 Livox 或 FAST-LIO
仓库，也不需要初始化 ikd-Tree 子模块。构建环境为 Ubuntu 22.04 / ROS 2 Humble。

| 项目 | 工作区路径 | 上游提交 | 许可证 |
| --- | --- | --- | --- |
| [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) | `third_party/livox_sdk2` | `c0796f04c143143899c87a773d9f6b7136453c0b` | MIT，具体第三方声明见 `LICENSE.txt` |
| [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | `src/livox_ros_driver2` | `21445540f0d100dc86a7e6df312dd70bbdb4afdf` | MIT，见 `LICENSE.txt` |
| [FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2/tree/ros2) | `src/fast_lio` | `2fffc570a25d0df172720bac034fbdb6a13d2162` | GPL-2.0，保留源文件原有声明 |
| [ikd-Tree](https://github.com/hku-mars/ikd-Tree/tree/fast_lio) | `src/fast_lio/include/ikd-Tree` | `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4` | GPL-2.0，许可文件来源见 FAST-LIO 的 `UPSTREAM.md` |

SDK 和驱动来自远程已经验证 MID360S 数据链的干净 Git checkout。
FAST-LIO 及展开的 ikd-Tree 已与对应上游提交逐文件核对。所有版权、许可文件
均保留；不纳入 `.git`、生成目录、设备日志或点云地图。

`third_party/livox_sdk2/COLCON_IGNORE` 使 colcon 跳过 SDK 包发现。
`livox_ros_driver2` 的 CMake 使用 `add_subdirectory` 在自己的构建目录内生成
带 PIC 的 `liblivox_lidar_sdk_static.a`，并私有链接到驱动共享组件
`liblivox_ros_driver2.so`。SDK 不再单独安装到 `/usr/local`，驱动也不再查找
`liblivox_lidar_sdk_shared.so`。SDK 在本集成中关闭共享库、示例和安装规则；
独立调用 SDK CMake 时仍保留其上游默认行为。

驱动默认使用 ROS 2 的清单和构建分支，不需要传入 `ROS_EDITION` 或
`DISTRO_ROS`。移除无调用的 APR 和 Git 依赖；`build.sh` 改为不清空整个工作区、
不替换清单的 Humble 包构建入口。测试直接加载驱动共享组件，调用内嵌 SDK
版本 API，并检查 CustomMsg 序列化及动态依赖。修正 SDK 内 RapidJSON 的
GCC 诊断选项拼写，以及驱动双回波转换中的未使用变量，消除编译警告。

FAST-LIO 的算法和传感器处理逻辑沿用上述固定版本。工作区适配包括统一
C++17、明确线程/OpenMP 与 ROS 依赖，以及将可选运行日志移出源码目录。
详细改动和日志参数见 [FAST-LIO 来源记录](../src/fast_lio/UPSTREAM.md)。

完整构建和启动方式见 [根目录 README](../README.md)。ROS 功能包放在 `src/`，
地图放在根目录 `maps/`；地图数据、构建产物与运行日志不提交。
