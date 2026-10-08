# FAST-LIO2 来源与工作区适配

此目录保留现场已验证的 ROS 2 FAST-LIO2 源码，并作为普通 `ament_cmake`
功能包参与根目录的 `colcon build`。算法实现未升级到另一分支或版本。

| 内容 | 上游来源 | 固定版本 |
| --- | --- | --- |
| FAST-LIO ROS 2 | <https://github.com/Ericsii/FAST_LIO_ROS2>，`ros2` 分支 | `2fffc570a25d0df172720bac034fbdb6a13d2162` |
| `include/ikd-Tree` | <https://github.com/hku-mars/ikd-Tree>，`fast_lio` 分支 | `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4` |

整合前，远程 `fastlio_ws/src/FAST_LIO_ROS2` 的普通源文件与上述
FAST-LIO 提交逐文件字节一致；展开的 ikd-Tree 三个文件也与对应 gitlink
提交逐文件字节一致。源码直接纳入本仓库，不依赖额外克隆或初始化子模块。
不保留上游 `.git`、运行日志和点云地图。

`LICENSE` 是上游随 FAST-LIO 提供的 GPL 第 2 版全文；各源文件原有的版权和
许可证声明均予保留。上游 `package.xml` 的 `BSD` 字段与仓库根许可证不一致，
此处改为 `GPL-2.0`。固定的 ikd-Tree 提交没有单独附带许可证文件；
`include/ikd-Tree/LICENSE` 补收录自同一上游首次加入 GPL 第 2 版许可证的提交
`81eaeee3b99bea34ab83138ae7ed2a10e1b05e1d`，没有因此更新 ikd-Tree 算法文件。

工作区适配仅涉及：

- 使用一致的 C++17 标准，保留 CMake 构建类型的编译选项；显式链接 OpenMP
  与线程库，移除没有实际调用的 Python C API 和 `pcl_ros` 依赖。
- 补齐 ROS 消息、服务、TF、Eigen、PCL 和 launch 的直接依赖声明。
- 保留 `/map_save` 服务和 `map_file_path` 参数。二维/三维地图由工作区根目录
  的 `maps/` 管理，系统启动文件负责传入完整 PCD 路径。
- 运行日志由 `runtime_pos_log_enable` 控制，默认关闭；开启后写入
  `runtime_log_dir`。默认目录依次取 `$ROS_HOME/fast_lio`、
  `$HOME/.ros/fast_lio` 或 `/tmp/fast_lio`，不写入源码目录。
  目录或文件无法打开时关闭运行日志并发出提示，退出时只关闭有效文件。

上游 README 保留用于参考。此工作区的雷达连接、启动和地图保存方式以根目录
README 及 `documents/navigation/mid360_fastlio2_deployment.md` 为准。
