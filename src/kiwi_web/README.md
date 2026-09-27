# 双底盘本地 Web 控制台

设备上运行一个 Python / ROS 2 服务，局域网浏览器访问 `http://设备IP:8080`。
HTML、JavaScript、CSS、字体回退和品牌图片均在设备本地，无 CDN、Node.js 运行服务、
rosbridge 或外部云服务依赖。Node.js 仅用于可选的浏览器自动化测试。

界面沿用用户提供的 `/home/hi/MSHEEP/web_frontend`：原始 `robot_ui.css`、品牌图片、
地图画布的绘制与交互代码，并接入当前活动底盘的 ROS 2 接口。保留建图、修图、导航、遥控、
设置页。履带默认使用 MID360S + FAST-LIO2：网页显示雷达内置 IMU、三维惯导里程计和二维投影扫描状态，地图画布仍是二维栅格；不渲染完整三维点云，也不展示未接入的相机、热成像、空气质量、地形规划、机器狗姿态和动作。

## 功能

| 页面 | 已接入的功能 |
| --- | --- |
| 建图 | 显式开始/停止 SLAM Toolbox；履带 MID360S 使用 FAST-LIO2、雷达内置 IMU 和二维投影扫描，Kiwi 或显式 `legacy` 模式保留 M10P；显示位姿和就绪状态、键盘遥控、停车保存二维 PGM/YAML 与三维 PCD（旧链路仅二维地图） |
| 修图 | 选择地图、停止运行模式、障碍/可通行/未知画笔、矩形框选、半径调整、撤销/重做、版本校验保存、导出 |
| 导航 | 加载地图、AMCL 初始位置与朝向、激光和规划路径、单点目标、真实任务反馈、取消、清理代价地图 |
| 多点路径 | 地图添加目标、数值编辑坐标/朝向、排序/删除、按地图保存路径、顺序执行、暂停/继续/跳过/停止 |
| 遥控 | W/S 前后、A/D 横移（仅 Kiwi）、Q/E 转向、按住屏幕方向键、速度比例、X 松控停车、T/空格软件急停 |
| 设置 | 二维地图导入/导出/删除、同名三维 PCD 单独下载、当前传感器链路、CPU/内存/磁盘/连接数、往返时延、ROS 诊断与启动日志 |

地图拖拽确定位置和朝向；右键平移，滚轮缩放。手机支持触摸绘制/选点、普通模式拖动和平移、
双指缩放及屏幕方向键。切页不会自动开始运动，也不会因另一个浏览器打开页面而重启机器人。
导航任务由设备运行，浏览器离开页面或断开连接后继续执行；**手动遥控**失焦、断线后停止。

## 安装和启动

```bash
cd /path/to/your/workspace
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-up-to kiwi_web \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 launch kiwi_web web.launch.py
```

默认监听 `0.0.0.0:8080`。用 `hostname -I` 查询设备 IP，然后在局域网电脑/手机中打开
`http://该IP:8080`。如果系统启用了防火墙，允许可信局域网访问 TCP 8080 即可；不需要向
浏览器开放 DDS 或额外 WebSocket 端口。IPv4/IPv6 地址跟随当前页面，不使用固定远端地址。

默认使用独立 `ROS_DOMAIN_ID=42`，避免同一局域网其他 ROS 项目的同名话题影响底盘。
服务启动的硬件、SLAM、Nav2 节点自动继承此域。命令行调试时先运行
`source tools/activate_workspace.sh`（完整工作区），只构建 Web 时先执行
`export ROS_DOMAIN_ID=42`；录包和 RViz 终端也要在同一个域。
可在 `web.yaml` 修改 `ros_domain_id`；如果启动前显式导出了 `ROS_DOMAIN_ID`，环境变量优先。

履带 MID360S 实机位于 `a@192.168.66.112`，已部署在 `~/ros2_ws_mid360`。启动 Web 前必须按根目录 README 的 MID360S 节依次加载 `/opt/ros/humble`、`~/livox_ws`、`~/fastlio_ws` 和 `~/ros2_ws_mid360` 四层环境；雷达经 `enp2s0=192.168.1.50/24` 连接 `192.168.1.117`，SSH 仍走 Wi-Fi。与命令行工具共用 `~/maps`：

```bash
ros2 launch kiwi_web web.launch.py maps_dir:="$HOME/maps"
```

首次打开 `http://192.168.66.112:8080` 时，服务仅提供网页，不启动传感器或底盘。可在 `web.yaml` 调整实机网络参数；Web 启动建图或导航前会校验网卡 IP/路由。该配置仅适用于这台实机，在另一台设备上使用前必须改成其实际地址。

直接运行并覆盖端口/地图目录：

```bash
ros2 run kiwi_web kiwi_web_server --host 0.0.0.0 --port 8080 \
  --maps-dir maps
```

默认地图目录为**当前安装所属工作空间的 `maps/`**，自动识别普通 colcon 隔离安装和
`--merge-install`，与启动命令所在目录无关。原有地图直接显示，不需要复制到用户目录。
`maps_dir` 和非空的参数文件覆盖路径如果是相对路径，均相对工作空间解析。
非标准安装布局通过 `workspace:=/absolute/workspace` 或 `--workspace` 明确指定。
外部 YAML 引用的图像必须在地图目录内。
支持标准二维 `trinary` 地图，包括非零原点朝向。浏览器修图不支持 `scale/raw` 模式地图。

启动服务时不启动硬件。点击“开始建图”“加载此地图”或“启动遥控模式”才启动所需 ROS 节点，
默认关闭 RViz。模式切换先归零速度、取消 action、退出本服务拥有的进程组，再启动新模式。
从建图切换到导航前先检查地图和活动底盘的导航参数；检查失败时保留当前建图进程。
不要同时用原来的导航 launch 或命令行键盘控制同一机器人；检测到旧节点时网页会拒绝重复启动。
`sensor_pipeline: auto` 按活动底盘选择：`tracked_pwm` 使用 MID360S + FAST-LIO2，`kiwi` 保留原 M10P 链路；需要临时运行旧履带 M10P 时显式设为 `legacy`。同一时间只能启动一套雷达/定位链路。切换活动底盘或传感器链路后要重启 Web 服务。履带横移上限为零，用户确认的速度换算值 `21.2` 已启用正常运动。Web 遥控和导航仍分别受实时传感器就绪、地图与参数链检查约束。
页面从服务端读取活动底盘类型和速度能力：履带禁用 A/D 横移，Kiwi 保留全向遥控。服务端速度仲裁也会拒绝履带横移指令。MID360S 的内置 IMU 进入 FAST-LIO2 内部惯性融合，保留的 C 板 IMU 仅作诊断和串口在线判据，不进入外部 EKF。

第一次建图流程：

1. 打开“建图”，点击“开始建图”。履带 MID360S 需等待雷达内置 IMU、FAST-LIO2 里程计、二维投影扫描和二维建图话题就绪；旧链路则按原有 IMU 标定和扫描门控等待。底盘在数据就绪前保持静止。
2. 确认场地安全和传感器就绪后，开启遥控并按住 W/S/Q/E 移动；Kiwi 还支持 A/D 横移。松开即停止。履带首次通过 Web 运动时观察低速起步、反向和转向。
3. 完成后松开控制并停车，填写新地图名并保存。MID360S 在建图进程仍运行时调用 FAST-LIO2 `/map_save`，校验 PCD 实际落盘，再保存同名二维 PGM/YAML；任一部分失败不会提示双地图保存成功。保存不会覆盖已有同名地图。大点云写盘期间 FAST-LIO2 可能短暂停止处理新扫描，因此务必先停车。
4. 需要修图时打开“修图”，选择地图，点击“打开修图”，编辑后“覆盖保存”。
5. 在“导航”加载经实际行驶建成的二维地图，选择“设置初始点”，拖拽箭头指定真实位置和朝向。FAST-LIO2 是局部里程计，AMCL 负责二维地图中的全局重定位；三维 PCD 使用本次 FAST-LIO2 的局部原点，不能直接当作二维 `map` 坐标中的地图。
6. 确认激光与地图对齐后，选目标并点击“开始导航”，或编辑保存多点路径再开始。

## 运行配置与访问密码

所有默认参数保留在所属功能包内，Web 包只管理服务配置和调用入口：

| 所属包 | 参数文件 | 使用者 / 传递路径 |
| --- | --- | --- |
| `kiwi_platform` | `config/active_platform.yaml`、`config/profiles/*.yaml` | 全局底盘选择及各链路文件映射 |
| `kiwi_web` | `config/web.yaml` | HTTP/WS 服务、地图目录、遥控上限、可选文件覆盖 |
| `kiwi_base` | `config/kiwi_base.yaml`、`config/tracked_pwm.yaml` | 所选底盘、EKF、键盘及 Web 物理限速 |
| `kiwi_slam` | `config/mid360_mount.yaml`、`config/mid360_fastlio.yaml` | MID360S 安装外参、FAST-LIO2 内置惯导参数；旧链路不使用 |
| `kiwi_slam` | `config/mapping_gate.yaml`、`config/tracked_mapping_gate.yaml` | 所选建图门控 |
| `kiwi_slam` | `config/slam_toolbox.yaml`、`config/tracked_slam_toolbox.yaml`、`config/tracked_rf2o.yaml` | 建图与履带激光里程计 |
| `kiwi_navigation` | `config/nav2.yaml`、`config/tracked_nav2.yaml`、`config/mid360_tracked_nav2.yaml` | 所选 AMCL、控制器、代价地图和导航行为树 |

`web.launch.py → Web 服务 → Supervisor → stack.launch.py` 传递相同的 `platform_file`、底盘和传感器链路标识，并完整传递
`base_params_file`、`lidar_params_file`、`slam_params_file`、`mapping_gate_params_file`、
`monitor_params_file`、`nav2_params_file`；MID360S 还传递网卡/IP、安装外参、FAST-LIO2 参数和 PCD 输出路径。空覆盖值由活动底盘档案解析，
不会把空字符串传给 ROS 参数加载器。Nav2 节点编排位于 `kiwi_navigation/navigation_core.launch.py`，
该入口只负责定位导航，独立于 Web；Web 指定速度输出话题以接入仲裁。原有 RViz 启动入口保留。

参数优先级：命令行 / launch 显式覆盖 > 所选 `config` > 活动底盘档案 > 包内 `web.yaml`；
算法参数不复制进 Web 配置。`base_params_file` 对 Web 物理限速检查、底盘和 EKF 同时生效。
MID360S 不启动外部 EKF；同一底盘文件仍控制 Web 限速和实际底盘。自主导航仍使用 `kiwi_navigation` 中的速度和加速度配置。

```bash
# 在目标设备修改各功能包内的配置，然后构建 / 重新构建。
# --symlink-install 时包内 YAML 修改会直接反映到安装目录。
ros2 launch kiwi_web web.launch.py

# 可选：覆盖某一个功能包的配置文件；相对路径从工作空间解析。
ros2 launch kiwi_web web.launch.py port:=8080 \
  slam_params_file:=src/kiwi_slam/config/slam_toolbox.yaml  # Kiwi 示例；履带需用专属文件

# 在这台 MID360S 实机上显式指定地图目录及网络参数时：
ros2 launch kiwi_web web.launch.py maps_dir:="$HOME/maps" \
  sensor_pipeline:=mid360s lidar_interface:=enp2s0 \
  host_ip:=192.168.1.50 lidar_ip:=192.168.1.117 lidar_model:=Mid360s
```

默认允许可信局域网内访问。需要密码时，在配置中设置 `access_token`，或在设备端设置
`KIWI_WEB_TOKEN` 环境变量。页面通过同源、HttpOnly、SameSite=Strict 的会话 cookie 使用
密码，不将密码存入前端源码、URL 或 localStorage。HTTP 是局域网明文传输；需要加密时可在
设备上配置 HTTPS 反向代理，同时转发 WebSocket 和原始 Host/Origin。

可选用户服务：在**目标设备**构建并 source 后生成匹配当前路径的服务文件。
服务只记录目标工作空间路径，运行配置仍读取各功能包；不复制 YAML 到 `~/.config/kiwi`。

```bash
ros2 run kiwi_web kiwi_web_install_service
systemctl --user daemon-reload
systemctl --user enable --now kiwi-web.service
journalctl --user -u kiwi-web.service -f
```

服务生成器也支持 `--workspace /absolute/workspace` 和 `--output /tmp/kiwi-web.service`。
用户服务在用户会话启动时启动；无人登录启动由设备管理员启用该用户的 linger。
安装服务前先退出前台启动的 Web 服务。同一 ROS 域只允许一个本地服务实例。

迁移时复制 `src/`、工作空间下的 `maps/`（包括 `.versions/`、`.routes/` 隐藏目录），
以及需要的项目工具与文档。不要跨设备搬运 `build/`、`install/`、`log/`；
在目标设备安装 ROS 2 Humble 依赖并重新构建。硬件串口名称和安装外参在所属功能包修改。
前端使用当前页面的 IP / 端口，不需要修改 JS；本机 IP、用户名和参考页面目录均不是运行依赖。

## 控制与通信契约

```text
Browser --same-origin WS--> CommandGate --/cmd_vel--> 旧链路底盘节点
                                    \--/kiwi_web/cmd_vel--> MID360 运动门控 --/mid360/cmd_vel_safe--> 履带底盘
Nav2 -> velocity_smoother --/kiwi_web/nav_cmd_vel--^
MID360S /livox/imu + /fastlio/odom_base + /scan_mid360 -> ROS bridge -> WS state
ROS /map /plan /tf + 二维投影扫描 -> HTTP PNG + WS state -> Browser
```

- 网页按 20 Hz 发送遥控，ROS 侧以 50 Hz 输出；收到松键/释放/急停请求即发布当前安全速度。
- 手动命令 250 ms 没有更新就归零，控制权租约 1 秒；设备发出的挑战码 300 ms 过期，
  防止积压的旧速度在网络恢复后被执行。浏览器超过 350 ms 未收到状态会释放控制权。
- 同时只有一个浏览器持有手动遥控权。导航模式不接受手动速度。重连、解除软件急停均不自动恢复运动。
- 软件急停锁定速度输出并取消导航；该功能是 ROS 软件停车，不替代断电或硬件急停。
- 旧链路中，Kiwi 轮速反馈或履带 RF2O 激光里程计，以及 EKF 超过 500 ms 未更新时禁止运动；旧履带还要求 C 板 IMU 样本新鲜。MID360S 链路使用 FAST-LIO2 里程计、雷达内置 IMU 和二维投影扫描的新鲜度作为主要定位门控，C 板原始 IMU 仅用于串口数据流在线判据，不参与定位融合。C 板 0x3B 帧不是电机 ACK，不能证明电机已执行命令。导航还要求 AMCL 定位 TF、扫描和 Nav2 状态就绪。
  导航中发生此类中断会锁定软件急停并取消任务，恢复数据后需人工解除并重新启动任务。
  底盘速度看门狗继续保留（Kiwi 500 ms、履带 200 ms），Web 进程异常退出时仍可使底盘归零。MID360S 额外的运动门控在里程计或扫描断流时持续向履带底盘输出零速。
- 每次模式/地图切换都会更换会话版本，其他浏览器在旧地图上产生的导航请求会被拒绝。
- 非本服务速度发布者若进入当前链路的命令话题，会显示冲突并抑制本服务运动。MID360S Web 使用私有 `/kiwi_web/cmd_vel`，外部普通 `/cmd_vel` 不能绕过网页仲裁进入这条链路；ROS 域内仍不应启动第二套底盘入口或向 `/mid360/cmd_vel_safe` 直接发速度。
- 状态与激光以 10 Hz 推送，激光最多 720 点、路径最多 1500 点；地图按版本通过 PNG 获取。
  WebSocket 不传整张 JSON 栅格、不传视频，慢客户端不积压速度，断开后可自动重连。
- 后端的地图文件操作/PNG 编码在线程中执行，独立于 ROS 控制循环；浏览器地图最高约 30 FPS，
  页面隐藏时停止绘制。地图上限 1600 万栅格、一次编辑 200 万栅格、每条路径最多 200 点。

设置页的时延为浏览器到设备的 WebSocket 往返时间，不等于电机响应时间。实际局域网时延和
建图/导航算力需在目标设备、无线链路和真实地图大小下测量；有线网络通常更稳定。

## 地图保存与恢复

地图名字只允许 1–64 位英文、数字、`_`、`-`。PGM 采用顶部开始的图像行，ROS 地图采用底部
开始的栅格行，前后端明确进行翻转，并正确处理 `origin[2]` 的旋转。

每次保存先持久化 `.versions/<name>_<id>.pgm` 和同版本 YAML，再原子替换根目录的
`<name>.yaml`，避免中途断电产生新旧元数据混搭。修图用 SHA-256 版本校验，拒绝覆盖另一
页面刚刚保存的版本。未知栅格固定为 205，配套 `free_thresh: 0.196`，避免重新加载成空闲。

历史版本保存在地图目录的 `.versions/`；删除是移走地图 YAML，并将同名 PCD 和路径一同归档，保留历史像素。
恢复时先停止运行模式，将所需历史 YAML **复制回地图根目录并命名为原地图名**，其相对图像
路径会正确指向 `.versions/`。导出的 ZIP 包含普通 PGM、YAML 和该地图的 `.routes.json`，
可用于备份。导入界面读取 PGM/PNG + YAML；导出的路径 JSON 可由管理员还原到 `.routes/`。
历史版本不自动清理，设置页显示磁盘占用。MID360S 的 `<name>.pcd` 可在设置页单独下载，避免将大点云读入现有二维地图 ZIP 的内存缓冲。Web 保存点云只校验文件结构和实际写盘，不保证点云几何精度；需要实车轨迹和外部真值验收。

## 验证

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select kiwi_platform kiwi_base lslidar_driver kiwi_slam kiwi_navigation kiwi_web \
  --event-handlers console_direct+
colcon test-result --verbose
```

`test_control.py` 覆盖速度限幅、控制权、过期报文、断线/看门狗和软件急停；`test_maps.py`
覆盖 PGM/ROS 坐标、未知栅格、非零原点朝向、事务版本、非法路径和导入导出；`test_http.py`
使用真实 HTTP/WebSocket 验证接口、鉴权和多个浏览器；`test_ros_integration.py` 在
localhost 的 ROS 域 216、221 和 223 中创建测试发布者，验证原 Kiwi/M10P、履带与 MID360S 链路的反馈/IMU 就绪链及真实 ROS action server，不启动硬件。MID360S 新路径还需实机静止联调。
`test_pointcloud.py` 与 `test_web_pointcloud.py` 验证 PCD 结构、同名覆盖保护、二维/三维联合保存失败处理及删除归档；格式校验不等于几何精度验收。

浏览器流程测试（两个终端，测试服务仅绑定 localhost:8081、使用 ROS 域 217）：

```bash
# 终端 1：先 source ROS 和工作区
python3 src/kiwi_web/test/browser_fixture.py

# 终端 2：Playwright 不属于设备运行依赖
npm install --prefix /tmp/kiwi-web-browser playwright
NODE_PATH=/tmp/kiwi-web-browser/node_modules node src/kiwi_web/test/browser_e2e.cjs
```

履带页面可用 `KIWI_BROWSER_CHASSIS=tracked_pwm python3 src/kiwi_web/test/browser_fixture.py`
单独预览，端口为 `8083`、ROS 域为 `222`，同样不连接硬件。
需要只预览 MID360S 文案和状态卡时，加上 `KIWI_BROWSER_PIPELINE=mid360s`；该模拟器仍使用测试传感器，不能代替实机数据链验收。

测试使用本机 `/usr/bin/google-chrome`，可通过 `CHROME` 指定其他 Chromium 可执行文件。
截图默认保存到 `/tmp/kiwi-web-screenshots`，`KIWI_SCREENSHOTS` 可覆盖。
此测试验证真实浏览器 + Web 服务 + ROS action 链路；测试地图和传感器是隔离的测试数据，
不能替代实车的串口、运动方向、定位精度、避障、无线断联和持续运行验收。

验证真实 Nav2 启动与速度重映射（localhost ROS 域 218，不启动硬件、不发送导航目标）：

```bash
python3 src/kiwi_web/test/validate_nav2_runtime.py
# 同一脚本以 MID360S 的二维扫描/里程计参数测试 Web Nav2 重映射：
KIWI_NAV_PIPELINE=mid360s python3 src/kiwi_web/test/validate_nav2_runtime.py
```

两种参数链都检查 map_server、AMCL 和六个 Nav2 节点均进入 active 状态，控制器/行为服务速度进入
`/kiwi_web/nav_raw`，平滑器输出 `/kiwi_web/nav_cmd_vel`，不存在绕过仲裁的 `/cmd_vel` 发布者。

真实进程启停检查（localhost ROS 域 219，串口明确指向测试目录中不存在的文件）：

```bash
python3 src/kiwi_web/test/validate_supervisor_runtime.py
```

该旧链路脚本分别启动遥控和建图进程，确认底盘/EKF/SLAM 节点存在，然后停止并检查无存活子进程。


完整控制链路联调（localhost ROS 域 220，预计约一分钟）：

```bash
python3 src/kiwi_web/test/validate_full_runtime.py
```

该旧链路脚本运行真实 Web HTTP/WS、原底盘 C++ 驱动、EKF、URDF、建图门控、SLAM Toolbox、
map_server、AMCL 和 Nav2，只以 PTY 串口固件和房间 LaserScan 替代硬件。测试包含三轴
遥控串口值、超时停车、实际建图保存、修图后三类栅格的全图比对、AMCL 初始定位、
真实两点导航、暂停/继续、雷达断流锁停及原有地图加载。原有地图仅只读加载，不改动文件。
测试复制参数到临时目录，以不同取值验证 SLAM / 门控 / 诊断 / Nav2 的参数覆盖实际生效；
不更改生产配置。日志保留在脚本打印的 `/tmp/kiwi-full-runtime-*`。

测试 ROS 域使用 216–220，与原底盘测试的随机 100–199 范围分离，避免并行验证时串话。
