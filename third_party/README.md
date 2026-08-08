# 第三方本地依赖

该目录保存由 `COLCON_IGNORE` 排除、不参与 ROS 2 `colcon` 扫描的上游源码。不要把这里的工程移动到
`src/`，也不要直接修改上游代码；确需修补时，应记录补丁和对应版本。

## Livox-SDK2

- 本地路径：`third_party/Livox-SDK2`
- 上游仓库：`https://github.com/Livox-SDK/Livox-SDK2.git`
- 固定版本：`v1.3.1`
- 固定提交：`f5d9375f84efe2b15bc0a052d3e18482ed13adf4`
- 用途：Mid-360S 网络通信、设备控制、点云和内置 IMU 数据接收
- ROS 接口：无；ROS 2 话题由 `livox_ros_driver2` 提供

下载目录已经由工作区根目录的 `.gitignore` 排除。重新获取时使用：

```bash
cd /home/ym/ros2_ws
git clone --branch v1.3.1 --depth 1 \
  https://github.com/Livox-SDK/Livox-SDK2.git \
  third_party/Livox-SDK2
```

只编译、不安装：

```bash
cmake -S third_party/Livox-SDK2 \
  -B third_party/Livox-SDK2/build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build third_party/Livox-SDK2/build -j
```

系统安装会写入 `/usr/local`，应在确认工控机环境后单独执行：

```bash
sudo cmake --install third_party/Livox-SDK2/build
sudo ldconfig
```
