# base

`base` 是工作区中保留的历史 ROS 2 接口包，目前只生成 `base/srv/Control.srv` 服务接口。
它不参与当前 Kiwi 底盘、E110、SLAM Toolbox、AMCL 或 Nav2 的运行链路，也不需要在建图和
导航流程中单独启动节点。

当前主链路请使用：

```text
kiwi_base -> kiwi_slam -> kiwi_navigation
```

如需兼容旧工程，可以构建该接口包：

```bash
cd /home/a/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select base
```

