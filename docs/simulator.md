# 用 rm_aim_sim 在线调试自瞄

本文说明如何让本仓库的自瞄（检测 → PnP → 跟踪 → IMM → Aimer → Planner → Shooter）
直接跑在 `rm_aim_sim` 仿真上，并在仿真里闭环控制云台、开火和统计命中。

`rm_aim_sim` 是独立的 Godot 仿真进程，本仓库不包含它。仿真侧文档见
[`../../rm_aim_sim/docs/`](../../rm_aim_sim/docs/)（协议、接入指南、配置参考）。

> 注意：`io/simulator/` 是另一套 daedalus 共享内存相机接口，与本接口无关。
> 本文新增的接口统一放在 `io/rm_aim_sim/`，yaml 中写作 `camera_name: "rm_aim_sim"`。

## 1. 接口设计

自瞄保持“应用层只依赖 `io` 抽象”的结构，只替换相机和 C 板两处：

```text
                       Sc_vision (standard_sim)
  ┌──────────────────────────────────────────────────────────┐
  │ YOLO → Solver(PnP) → Tracker/IMM → Aimer → Planner       │
  │                                      ↘ Shooter           │
  │        ▲ q / mode / bullet_speed          │ Command      │
  │        │                                  ▼              │
  │  io::Camera                          io::rm_aim_sim::CBoard
  │  camera_name: rm_aim_sim                  │              │
  └──────────┬────────────────────────────────┼──────────────┘
             │ frame-*.raw + frame.json       │ command.json
             │ request.json                   ▼
  ┌──────────┴────────────────────────────────────────────────┐
  │ rm_aim_sim（Godot）: state.json / 虚拟相机 / 云台 / 弹道   │
  └───────────────────────────────────────────────────────────┘
```

新增文件：

| 文件 | 作用 |
|---|---|
| `io/rm_aim_sim/rm_aim_sim_client.{hpp,cpp}` | rm_aim_sim/2 本机共享内存协议客户端：state 轮询、图像请求/读取、指令发布、序号与超时处理 |
| `io/rm_aim_sim/rm_aim_sim_camera.{hpp,cpp}` | `CameraBase` 实现，接入 `io::Camera` 的 `camera_name` 分支 |
| `io/rm_aim_sim/rm_aim_sim_cboard.{hpp,cpp}` | `io::CBoard` 的仿真版：`state.json` 当反馈，`command.json` 当控制 |
| `src/standard_sim.cpp` | 自瞄仿真入口，算法模块与 `standard_mpc_se` 完全一致 |
| `configs/sim.yaml` | 与 rm_aim_sim 默认相机/装甲/弹速配套的配置 |

与实车接口的对应关系：

| 实车 | 仿真 |
|---|---|
| `io::Camera`（mindvision / hikrobot） | `io::Camera`，`camera_name: rm_aim_sim` |
| `/dev/ttyACM*` 上的 `io::CBoard` | `io::rm_aim_sim::CBoard` |
| 下位机上报 `Autoaim_s` | `state.json` 中的 `gimbal/weapon/ipc` |
| 下发 `GimbalControl` | `command.json` 中的 `yaw/pitch/control/fire` |
| 相机时间戳（steady_clock） | 每帧拷贝完成后的本地 `steady_clock` 时刻 |

`Client` 通过 `Client::instance(channel)` 在同一进程内共享，因为仿真同一通道只允许
一个客户端（`client.lock`）。相机和 C 板适配器不能各自新建连接。

## 2. 编译

依赖与主工程相同（OpenCV、Eigen、yaml-cpp、nlohmann-json、OpenVINO 等），没有新增
第三方库。`io/CMakeLists.txt` 已经加入新源文件。

如果 `build/` 已配置过，直接编译目标：

```bash
cd /home/omup/code/Sc_vision
cmake --build build --target standard_sim -j$(nproc)
```

从零配置时（本工程依赖 ROS 2 的 `serial_driver` 查找，若未安装需先 source ROS）：

```bash
cd /home/omup/code/Sc_vision
source /opt/ros/jazzy/setup.bash      # 按本机 ROS 发行版调整
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target standard_sim -j$(nproc)
```

产物：`build/standard_sim`。

## 3. 运行

### 3.1 终端一：启动仿真

```bash
cd /home/omup/code/rm_aim_sim
./run.sh -- --remote --config=res://configs/spin.cfg
```

- `--remote` 等价于打开界面里的“本机自瞄控制”。没有它，`state.ipc.enabled=false`，
  自瞄会保持 `idle` 并只发停控指令。
- `spin.cfg` 让靶车以大约 4 rad/s 自转，便于验证跟踪和提前量。静止靶可以先按
  README 的默认配置验证 PnP，但只看到一块装甲时跟踪器更容易发散（见第 7 节）。
- 必须使用图形模式。`--headless` / `--no-ipc` 没有图像，`standard_sim` 会等待帧超时。

### 3.2 终端二：启动自瞄

```bash
cd /home/omup/code/Sc_vision
./build/standard_sim configs/sim.yaml
```

首次联调保持 `configs/sim.yaml` 中：

```yaml
sim_allow_fire: false
```

确认云台能稳定跟随、命令角与反馈角一致后，再改为 `true` 或使用临时配置：

```bash
sed 's/^sim_allow_fire: false/sim_allow_fire: true/' configs/sim.yaml > /tmp/sim_fire.yaml
./build/standard_sim /tmp/sim_fire.yaml
```

退出：仿真窗口按 `Q`/`Esc`，或终端 `Ctrl+C`。`standard_sim` 也会在窗口 `Q`/`Esc` 退出。

> 同一通道只允许一个客户端。`standard_sim` 运行时不要再启动
> `tools/control_demo.py`、`tools/camera_demo.py` 或 `examples/cpp/sim_client_demo`，
> 否则会报 `client.lock` 冲突。

## 4. 配置说明

`configs/sim.yaml` 由 `standard.yaml` 派生，区别集中在 IO 与仿真标定：

```yaml
camera_name: "rm_aim_sim"

rm_aim_sim:
  channel: "default"       # 必须与仿真 [ipc].channel 一致
  connect_timeout_ms: 8000 # 等待共享目录的时间

sim_allow_fire: false      # standard_sim 专有：首次联调不开火
show_debug_window: true
debug_window_scale: 0.5
```

| 键 | 作用 |
|---|---|
| `camera_name` | `rm_aim_sim` 时由 `io::Camera` 创建仿真相机；其他值保持原行为 |
| `rm_aim_sim.channel` | 共享内存通道，目录为 `/dev/shm/rm_aim_sim_<channel>/` |
| `rm_aim_sim.connect_timeout_ms` | 启动时等待仿真目录的时间 |
| `sim_allow_fire` | 为 false 时强制 `command.shoot=false`，只瞄准不开火 |
| `show_debug_window` / `debug_window_scale` | 调试窗口开关和缩放 |
| `use_default_bullet_speed` | false 时使用 `state.weapon.speed`；true 时固定用 yaml 的 `bullet_speed` |

仿真默认标定（`configs/sim.yaml` 已按此填写）：

| 项目 | 值 |
|---|---|
| 分辨率 / 垂直 FOV | 1440 × 1080 / 45° |
| `fx = fy` | `1303.6753236814714` |
| `cx, cy` | `720, 540` |
| 畸变 | `[0, 0, 0, 0, 0]` |
| 光心在云台系偏移 | `[0.08, 0, 0.04]` m |
| `R_camera2gimbal` | `[0,0,1, -1,0,0, 0,-1,0]` |
| 小/大装甲灯条间距 | 0.135 m / 0.230 m |
| 灯条长度 | 0.056 m |

仿真世界系：x 前、y 左、z 上，地面 z=0；yaw 左为正，**pitch 向下为正**；
控制指令是绝对目标角，不是增量。`Solver` 求出的位置以云台旋转中心为原点，
仿真真值以地面为原点，两者对比时要减去 `gimbal.origin`（默认 `[0,0,0.65]`）。
角度指令不受这个原点差异影响。

如果改了仿真的分辨率、FOV 或相机安装偏移，必须同步修改 `camera_matrix`、
`distort_coeffs`、`t_camera2gimbal`。首次取图时 `RmSimCamera` 会打印：

```text
[RmSimCamera] 首帧 1440x1080，fx=1303.675，cx=720.000，cy=540.000
[RmSimCamera] yaml 内参与仿真相机一致
```

出现“yaml 内参与仿真不一致”的警告时，不要继续调跟踪，先对齐标定。

## 5. 接口行为与保护

### 5.1 状态与模式

`io::rm_aim_sim::CBoard::refresh()` 每个主循环调用一次，从最新 `state.json` 更新：

- `mode`：`ipc.enabled && !paused && state 新鲜` 时为 `auto_aim`，否则 `idle`。
  仿真没有小符/大符/前哨站模式。
- `bullet_speed`：默认取 `weapon.speed`。
- `feedback_yaw/pitch/yaw_vel/pitch_vel`：取 `gimbal` 实际值，用于调试曲线。
- `stats`：`fired/hit/resolved/in_flight/hit_rate/damage/kills`，用于窗口和曲线。
- `session_id`：暂停/恢复、重置、外部控制切换都会变化。

`standard_sim` 在 session 变化时会重建 Tracker/Aimer/Planner/Shooter 并清空上一条指令，
避免把旧目标带进新会话。仿真重启（publisher 改变）也会触发同样的清理。

### 5.2 图像请求

`read_frame()` 保证同时最多一个未完成请求：

1. 读取新鲜状态，向 `request.json` 原子写入唯一 `request_id`；
2. 轮询 `frame.json`，校验 publisher、session、sim_time、宽高、编码、文件名、字节数；
3. 打开 `frame-<request_id>.raw`，读入自己的 BGR `cv::Mat`；
4. 清除请求，下一帧再发新请求。

默认等待超时 5 秒，超时抛出异常并提示“仿真必须使用图形模式”。正常运行时不会积压
帧；处理慢时帧率下降，但不会读到半帧。

### 5.3 控制与安全

- `send()` 会自动带上最新 `publisher_id/session_id`，序号严格递增，并从
  `ipc.last_seq+1` 继续。
- 状态不新鲜、暂停或外部控制关闭时，`control` 自动置 false；`fire` 只在 `control`
  有效时为 true。
- 仿真端指令看门狗默认 250 ms。若检测推理导致发指令间隔超过 250 ms，云台会停控；
  此时应降低分辨率、提高检测速度，或改成独立的控制线程。
- 进程退出时客户端会尽力发送 `control=false, fire=false`；但安全兜底仍依赖仿真看门狗。

### 5.4 时间与线程

- `state.json` 由后台线程每 2 ms 轮询一次，只在 `publication` 更新时刷新快照。
- 相机帧时间戳使用客户端 `steady_clock` 的接收时刻；`Tracker` 的 dt、Aimer 的
  `now - timestamp` 时延补偿都在这个时间域内。
- `imu_at(t)` 优先返回与当前帧同帧的 `gimbal.quaternion_wxyz`，避免云台运动时
  PnP 用了稍晚的姿态；没有同帧数据时按相邻状态样本 slerp。
- `CBoard::send()` 和 `Client::send()` 有锁保护，但 `read_frame()` 只应由采图线程调用；
  当前 `standard_sim` 是单线程流水线。

## 6. 验收顺序

1. **通信**：仿真运行、`sim_allow_fire: false`。启动后应看到：
   ```text
   [RmSimCamera] 已连接 rm_aim_sim 通道 default
   [RmSimCBoard] 已连接 rm_aim_sim 通道 default
   [RmSimCamera] 首帧 1440x1080 ...
   [standard_sim] mode -> auto_aim (fresh=true)
   ```
2. **控制**：手动在仿真里按方向键转动云台或开启内置跟踪，确认程序日志中的
   `feedback_yaw/pitch` 跟着变化；关闭界面“本机自瞄控制”后，程序应切回 `idle`。
3. **相机/PnP**：默认静止靶，画面上的重投影点应贴在装甲上；`plotter` 数据中
   `x≈4 m`、`z≈-0.39 m`（相对云台原点）。只看到一块装甲时角度可能有抖动，
   属于单板不可观测问题，可切自转靶再测。
4. **跟踪/控制闭环**：用 `spin.cfg` 或 `combined.cfg`，观察云台持续跟随、换向，
   `w`、`a` 估计与目标旋转方向一致；不射击时先确认没有明显甩枪。
5. **射击**：设置 `sim_allow_fire: true`，确认 `state.stats.fired` 增长，
   命中后 `hit`/`resolved` 更新。仿真界面显示 `命中率 = hit / resolved`。
6. **异常**：
   - 仿真按 `P` 暂停/恢复：session 变化，程序应重建跟踪状态；
   - 仿真按 `R` 重置：同样清空旧目标；
   - 关闭仿真：程序等待下一帧 5 秒后报错退出，不会继续用旧图像控制；
   - 重启仿真：重新启动 `standard_sim`（当前不做自动重连）。

一个参考结果（旋转靶、`spin.cfg`、15 秒、自动开火）：
`fired=58, hit=19, hit_rate=32.76%`。命中率取决于检测质量、目标转速和火控参数，
这个数字只说明链路已闭环，不是算法性能上限。

## 7. 常见问题

| 现象 | 排查 |
|---|---|
| `等待仿真通道超时` | 仿真没启动、`[ipc].channel` 与 `rm_aim_sim.channel` 不一致、不是同一用户 |
| `无法锁定 client.lock` | 同通道已有 `control_demo.py`/`camera_demo.py`/其他自瞄在运行 |
| `等待相机帧超时` | 仿真用了 `--headless`/`--no-ipc`，或仿真已关闭；必须图形模式 |
| 云台不动 | 界面“本机自瞄控制”是否开启（`ipc.enabled`）、是否暂停、日志是否一直 `idle` |
| yaml 内参不一致警告 | 仿真分辨率/FOV 改过；同步 `camera_matrix` 和畸变，或恢复默认 |
| 重叠/红蓝反了 | `enemy_color` 与仿真靶车阵营；仿真默认 `team=blue` |
| 静止靶 `Target diverged` | 默认靶只露出一块装甲，r/l/h 不可观测；改用 `spin.cfg`，或降低 `min_confidence` 抖动、调整过程噪声 |
| 识别率低 | 仿真相机曝光增益（界面滑条/`[camera].brightness_gain`）、`min_confidence`、模型域差异；先提高曝光再调阈值 |
| 开火但打不中 | 先看命令角与反馈角、`pitch` 符号、弹速是否来自 `weapon.speed`；再看 `sim_allow_fire`、`auto_fire`、Shooter 容差 |
| 每隔一段时间停控 | 发指令间隔超过仿真 250 ms 看门狗；检查推理耗时和帧率 |
| 启动后马上退出 | 查看是否 `Large dt`、模型加载失败、OpenVINO 设备不可用；与仿真接口无关 |

## 8. 把仿真接进其他自瞄入口

`standard_sim` 已经展示了完整接法。若要在自己的入口里复用：

1. yaml 设置 `camera_name: "rm_aim_sim"`，用 `io::Camera` 采图；
2. 用 `io::rm_aim_sim::CBoard` 替换 `io::CBoard`；
3. 每个循环先 `camera.read()`，再 `cboard.refresh()`，然后 `imu_at(t)`/`mode`；
4. 检查 `session_id()` 变化并重建跟踪状态；
5. 用 `cboard.send(command)` 下发，`command.shoot` 是否真正开火由你的安全开关和
   Shooter 共同决定。

`io::rm_aim_sim::CBoard` 的公开字段（`bullet_speed/mode/shoot_mode/yaw_vel/pitch_vel`）
和 `feedback_*` 方法与 `io::CBoard` 对齐，现有自瞄上层逻辑可以平移过来；区别只是多了
`refresh()`、`session_id()` 和 `stats()`。它不是 `io::CBoard` 的子类，其他使用
`io::CBoard` 的入口需要显式替换类型。
