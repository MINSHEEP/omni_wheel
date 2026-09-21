# 本次验证记录

日期：2026-09-21。环境：Ubuntu 22.04、ROS 2 Humble、本机 Chrome。

## 构建和回归

```text
colcon build --symlink-install --packages-up-to kiwi_web --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
Summary: 6 packages finished

colcon test --packages-select kiwi_base lslidar_driver kiwi_slam kiwi_navigation kiwi_web --event-handlers console_direct+
colcon test-result --verbose
Summary: 150 tests, 0 errors, 0 failures, 0 skipped
```

包含新增 21 项 Python 测试；后续边界修复后重新运行 kiwi_web 测试均通过。
新增 JavaScript 已通过 `node --check`，Python 通过 flake8 的 F 类错误检查。

关键行为覆盖：遥控租约、速度限幅、超时归零、过期与乱序报文、多浏览器互斥、软件急停、
地图坐标翻转、未知区域、透明 PNG、编辑版本冲突、保存事务中断、路径持久化、同源/密码、
真实 ROS action 顺序执行、目标尚未确认时取消、暂停继续、底盘反馈断流停车。
追加检查旧模式/地图请求被拒绝、非对象 WebSocket 报文被拒绝、导航反馈恢复后保持锁定停车。

## 浏览器

使用 `test/browser_fixture.py` 和 `test/browser_e2e.cjs`，测试 ROS 域 217。
通过以下实际页面操作：

- 建图模式、接收地图、保存、遥控控制权和失焦释放。
- 打开修图、画笔、矩形、撤销、重做、保存并验证文件版本与地图元数据。
- 非零地图原点旋转的屏幕/世界/栅格坐标转换与 PGM 索引。
- 初始位姿发布、单点导航、多点路径保存/开始/暂停/继续。
- 屏幕遥控、软件急停锁定与解除后不恢复运动。
- 地图文件上传导入、下载导出、删除。
- 五页桌面布局和 390×844 手机布局，无 JavaScript 页面错误、无横向溢出。

测试截图：`/tmp/kiwi-web-screenshots/`。这些截图中的地图和状态来自隔离测试环境。
正式服务也单独完成五个页面加载检查，未启动硬件。
正式服务重启后浏览器自动重新连接，服务会话更新且保持 idle/无遥控持有者。
独立浏览器验证另一客户端切换地图时，本页旧目标被清空，旧会话的导航请求返回拒绝，
不会启动基于旧地图的运动。完整浏览器流程在上述修复后重新通过。

## 真实 Nav2

`test/validate_nav2_runtime.py` 的等价验证在 ROS 域 218 使用测试 scan/odom/TF 运行，
未启动真实串口节点，未发送运动目标。

```text
map_server: active
amcl: active
controller_server: active
smoother_server: active
planner_server: active
behavior_server: active
bt_navigator: active
velocity_smoother: active
```

验证控制器和行为服务发布 `/kiwi_web/nav_raw`，速度平滑器发布 `/kiwi_web/nav_cmd_vel`，
不存在绕过网页仲裁的 `/cmd_vel` 发布者。退出时各子进程正常结束。

另在 localhost ROS 域 219 执行真实遥控/建图进程启停验证，底盘与雷达串口明确设置为
临时测试目录内不存在的文件，实际启动底盘/EKF/SLAM 等节点后由 Supervisor 停止，
确认两次均无存活子进程。可由 `test/validate_supervisor_runtime.py` 复现。

## 迁移与参数

- 默认地图目录已改为当前工作空间的 `maps/`，由 colcon 安装前缀推导，不依赖当前目录或用户名。
- 从 `/tmp` 启动实际 `web.launch.py port:=8082`，成功读取原工作空间四张地图，服务保持 idle。
- 默认和覆盖参数保留在各包；覆盖路径在启动子进程前解析，空覆盖使用子包默认。
- 6 个 `*_params_file` 从 launch 到服务到 Supervisor 到子包完整透传。
- Nav2 的节点、行为树和参数装配移至 `kiwi_navigation/launch/navigation_core.launch.py`；
  Web 只传地图、可选参数文件和速度话题。原有两个导航 launch 未改变。
- 服务生成器根据目标工作空间生成 systemd 文件，经 `systemd-analyze --user verify` 验证；
  不复制参数到用户目录，不依赖本机 IP、源码参考目录或固定工作空间名字。
- ROS 域默认 42，可由配置或明确导出的环境变量覆盖。
- 本机临时服务已停止并取消自启动；目标设备部署不依赖本机用户服务。
- 补齐导航包对 rotation_shim_controller、nav2_common、ament_index_python 的直接依赖。
  `rosdep check --from-paths src --ignore-src --rosdistro humble` 仅提示本机缺少 apt 版
  `python3-aiohttp`（本机测试使用已安装的 Python aiohttp）。目标设备按 README 的
  `rosdep install` 安装系统依赖。

## 完整真实节点联调

`test/validate_full_runtime.py` 使用 ROS 域 220，串口明确为 PTY，雷达串口指向临时目录下
不存在的文件（真实雷达驱动预期退出），测试节点发布合成房间扫描。
真实底盘驱动接收模拟板卡的电机 / IMU 反馈，原 EKF 计算里程计；没有伪造 map→odom，
也没有用假 Action Server 代替导航。校准样本数量和传感器融合配置沿用原包。

已经通过：

- WebSocket 的前进、横移、转向三轴速度，经过 `/cmd_vel` 和底盘运动学变为串口 `0x3A`。
- Web 指令超时后串口输出归零。
- 原 IMU 校准、原扫描门控、真实 SLAM Toolbox 产生地图，网页保存 PGM/YAML。
- 网页修图后由真实 map_server 读取；整张地图逐格比对，包括新增障碍、自由和未知栅格。
- 网页初始位姿由真实 AMCL 接收，定位 TF 建立后 Nav2 激活；网页激光层取得 720 个点。
- Nav2 两点路线真实返回成功；网页暂停取消 Action 且串口归零，继续后完成剩余点。
- 雷达断流后锁定急停，串口归零；恢复扫描后不自动恢复运动。
- 原有 ceshi1、ceshi2、ceshi3、warehouse 地图均成功由真实 map_server 加载，与网页像素逐格一致。
- 查询实际节点参数，确认底盘/EKF 参数、SLAM 分辨率、建图门控样本数、诊断设置、Nav2 控制频率
  均正确传入；静态导航地图使用静态诊断模式。

本次复查修复了：诊断 byte 无法 JSON 序列化、遗漏 SLAM 专用诊断话题、激光先于 TF 到达时
整帧丢弃、Nav2 未激活却显示可导航、退出进程组重复发送 SIGINT 干扰 DDS 清理、切换模式时
旧节点发现缓存误报冲突、空 launch 参数覆盖子包默认值、静态地图误报更新超时。

最终完整联调记录：`/tmp/kiwi-full-runtime-result.log`，节点日志目录
`/tmp/kiwi-full-runtime-ox0sm97u/`，结束标记 `FULL_RUNTIME_PASS`。
目标点 1.0 m、1.4 m 依次成功；模拟板卡最终 x≈1.232 m、y≈−0.047 m。
这些数值仅证明模拟闭环可完成动作，不能当作真实定位误差指标。

全量回归曾发生测试域碰撞：原底盘测试随机使用 100–199，浏览器测试原先固定 184，
导致“未注入反馈却收到里程计”断言失败。新增测试已迁至独立 216–220 范围后重测。

## 本机参考性能

本机无图、无硬件运行时的参考测量：100 次本机 WebSocket ping，往返中位数约 0.041 ms、
P95 约 0.121 ms；2 秒样本中服务 CPU 约一个核心的 4%，两个 Python 进程 RSS 合计约
108 MiB，systemd 服务 cgroup 内存约 63 MiB。RSS 会重复计算共享页。这是短时间本机测量，
不代表无线链路、实际运动响应或满负载建图导航性能。

## 尚需实车验收

本机没有 `/dev/robomaster_c`、`/dev/ttyUSB*`、`/dev/ttyACM*`。因此未实测真实轮组运动、
激光地图质量、AMCL 精度、动态避障、实际无线延迟和满负载持续运行。
接入设备后按 README 的建图→保存→修图→定位→单点→多点流程验收，并现场验证按键松开、
浏览器失焦、网络断开和软件急停的停车表现。软件急停不能代替硬件急停。
