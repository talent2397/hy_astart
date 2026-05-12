# Hybrid A* 系统框架文档

> 面向新接触本项目的开发者。说明系统架构、数据流向、代码目录和配置修改位置。

---

## 一、系统架构总览

```
┌─────────────────────────────────────────────────────────────────┐
│                         RViz (可视化)                            │
│  2D Pose Estimate → /initialpose                                │
│  2D Nav Goal      → /move_base_simple/goal                     │
│  显示 /planned_path (绿色路径线)                                  │
│  显示 RobotModel (通过 TF)                                       │
└──────────────┬──────────────────────────────────┬───────────────┘
               │                                  │
               ▼                                  ▼
┌──────────────────────────────┐    ┌──────────────────────────────┐
│     Gazebo 仿真环境           │    │       map_server             │
│  test.world (障碍物+走廊)     │    │  加载 map_gazebo.yaml        │
│  car.urdf.xacro (4轮小车)    │    │  发布 /map (ROS topic)        │
│  diff_drive 差速插件         │    │                               │
└──────────────┬───────────────┘    └──────────┬───────────────────┘
               │                               │
               │ /gazebo/model_states           │ /map
               │ (100Hz, 真实位姿+速度)          │ (启动时1次)
               ▼                               ▼
┌─────────────────────────────────────────────────────────────────┐
│              进程1: run_gazebo_bridge (100Hz)                     │
│                                                                   │
│  ROS → SHM:  车辆状态、目标点、初始位姿、地图                       │
│  SHM → ROS:  cmd_vel (发给diff_drive)、/planned_path (给RViz)     │
│  发布 TF:     odom → base_link (Gazebo真实位姿)                    │
└──────┬──────────────────┬──────────────────┬─────────────────────┘
       │ 写 SHM           │ 读 SHM           │ 读 SHM
       ▼                  ▼                  ▼
┌─────────────┐  ┌──────────────┐  ┌──────────────────┐
│ vehicle_state│  │  planner_path │  │   control_cmd    │
│ (100Hz)      │  │  (~1Hz 按需)  │  │   (100Hz)        │
│ goal_pose    │  │               │  │                   │
│ initial_pose │  │               │  │                   │
│ map_data     │  │               │  │                   │
└──────┬───────┘  └──────┬───────┘  └────────┬──────────┘
       │ 读              │ 写                │ 读
       ▼                 ▼                   │
┌──────────────────┐  ┌──────────────────────────────┐
│ 进程2: run_planner│  │  进程3: run_tracker (100Hz)  │
│     (10Hz)        │  │                              │
│                   │  │  读 vehicle_state (SHM)       │
│  读 vehicle_state │  │  读 planner_path (SHM+M通知)  │
│  读 goal_pose     │  │  写 control_cmd (SHM)         │
│  读 map_data      │  │                              │
│  读 initial_pose  │  │  Pure Pursuit 算法:           │
│                   │  │  找最近点 → 前视点 → 转向角    │
│  Hybrid A* 搜索:  │  │  曲率自适应速度 → 终点判定     │
│  RS曲线捷径       │  │                              │
│                   │  │                              │
│  写 planner_path  │──▶ 通知 Tracker (MQ_PATH_READY)  │
│  发 PATH_READY    │  │                              │
└──────────────────┘  └──────────────┬───────────────┘
                                    │
                                    │ control_cmd (SHM)
                                    ▼
                          ┌─────────────────┐
                          │ run_gazebo_bridge│
                          │ 读 control_cmd   │
                          │ 计算 cmd_vel:    │
                          │  v = ∫accel·dt   │
                          │  ω = v·tan(δ)/L  │
                          │ 发布 /cmd_vel    │
                          └────────┬────────┘
                                   │ /cmd_vel (ROS topic)
                                   ▼
                          ┌─────────────────┐
                          │ diff_drive 插件  │
                          │ 左右轮差速驱动   │
                          │ 小车在Gazebo运动 │
                          └─────────────────┘
```

**核心原则：ROS 仅用于 Gazebo 接口层，进程间核心数据全部走共享内存。**

---

## 二、进程间通信 (IPC) 详解

### 2.1 共享内存 (SHM) — 高频大数据

共 6 个 SHM 段，全部在 `/dev/shm/` 下：

| SHM 段 | POSIX 名 | 数据结构 | 写入方 | 读取方 | 频率 |
|--------|----------|----------|--------|--------|------|
| vehicle_state | `/hy_astar_vehicle_state` | VehicleStateData | Bridge | Planner, Tracker | ~100Hz |
| planner_path | `/hy_astar_planner_path` | PlannerPathData | Planner | Tracker, Bridge | ~1Hz |
| control_cmd | `/hy_astar_control_cmd` | ControlCmdData | Tracker | Bridge | ~100Hz |
| goal_pose | `/hy_astar_goal_pose` | GoalPoseData | Bridge | Planner | 按需 |
| initial_pose | `/hy_astar_initial_pose` | GoalPoseData | Bridge | Planner | 按需 |
| map_data | `/hy_astar_map_data` | MapShmHeader + grid[] | Bridge | Planner | 启动时 |

**同步机制**：每个 SHM 段有 `is_valid` (uint8_t) + `sequence` (uint64_t)。写入时：`is_valid=0` → memcpy → `sequence++` → `is_valid=1`。读取时检测 `sequence` 变化，无锁零拷贝。

### 2.2 消息队列 (MQ) — 低频事件通知

| MQ | POSIX 名 | 用途 | 发送方 | 接收方 |
|----|----------|------|--------|--------|
| PATH_READY | `/hy_astar_path_ready` | 新路径就绪通知 | Planner | Tracker |
| REPLAN_REQUEST | `/hy_astar_replan_request` | 重规划请求 | (预留) | Planner |
| EMERGENCY_STOP | `/hy_astar_emergency_stop` | 紧急停止 | (预留) | Tracker |

### 2.3 代码位置

| 组件 | 文件 | 说明 |
|------|------|------|
| SHM/MQ 名称和数据结构定义 | `src/ipc/include/ipc/data_protocol.h` | 改 SHM 段名、结构体字段、MQ 类型 |
| SHM 底层封装 | `src/ipc/include/ipc/shared_memory.h` | shm_open/mmap 封装 |
| MQ 底层封装 | `src/ipc/include/ipc/message_queue.h` | POSIX mq 封装 |
| 高层读写接口 | `src/ipc/include/ipc/data_bus.h` | DataBusWriter/Reader 模板 |

---

## 三、三个进程详解

### 3.1 run_gazebo_bridge — ROS ↔ SHM 桥接

**代码位置**：`src/gazebo_bridge/app/run_gazebo_bridge.cpp`（单文件）

**职责**：
- 订阅 `/gazebo/model_states` → 找到 "car" 模型 → 写入 `vehicle_state` SHM + 发布 `odom→base_link` TF
- 订阅 `/move_base_simple/goal` → 写入 `goal_pose` SHM
- 订阅 `/initialpose` → 写入 `initial_pose` SHM + 调用 Gazebo `set_model_state` 传送小车
- 订阅 `/map` → 写入 `map_data` SHM
- 读 `control_cmd` SHM → 积分加速度 → 发布 `/cmd_vel` (100Hz)
- 读 `planner_path` SHM → 转换为 `nav_msgs::Path` → 发布 `/planned_path` (给 RViz)

**关键参数**（文件内直接修改）：
- `kWheelBase = 2.0` — 自行车模型轴距（第 51 行）
- 速度上限 `3.0 m/s`（ControlLoop 内）
- 角速度上限 `±1.0 rad/s`

### 3.2 run_planner — Hybrid A* 路径规划

**代码位置**：
- 入口：`src/planner/app/run_planner.cpp`
- 接口：`src/planner/include/planner/i_planner.h` (抽象接口)
- 实现：`src/planner/include/planner/hybrid_a_star_planner.h` + `src/planner/src/hybrid_a_star_planner.cpp`
- 核心搜索：`src/planner/include/planner/hybrid_a_star.h` + `src/planner/src/hybrid_a_star.cpp`
- RS 曲线：`src/planner/include/planner/rs_path.h` + `src/planner/src/rs_path.cpp`

**职责**：
- 读 `vehicle_state` / `initial_pose` (起点) + `goal_pose` (终点) + `map_data` (地图)
- 调用 Hybrid A* 搜索（RS 曲线捷径，shot_distance=5m 内直接用 RS）
- 规划成功 → 写入 `planner_path` SHM → 发 `PATH_READY` MQ 通知 Tracker

**配置参数**（`run_planner.cpp` 第 30-39 行，硬编码）：

| 参数 | 值 | 含义 |
|------|-----|------|
| steering_angle | 15.0 | 最大转向角 (度) |
| wheel_base | 2.0 | 轴距 (米) |
| segment_length | 1.6 | 搜索步长 (米) |
| segment_length_discrete_num | 8 | 每步细分段数 |
| steering_penalty | 1.5 | 转向惩罚系数 |
| reversing_penalty | 3.0 | 倒车惩罚系数 |
| steering_change_penalty | 2.0 | 转向变化惩罚 |
| shot_distance | 5.0 | RS 捷径触发距离 (米) |

### 3.3 run_tracker — Pure Pursuit 轨迹跟踪

**代码位置**：
- 入口：`src/tracker/app/run_tracker.cpp`
- 接口：`src/tracker/include/tracker/i_tracker.h` (抽象接口)
- 实现：`src/tracker/include/tracker/pure_pursuit_tracker.h` + `src/tracker/src/pure_pursuit_tracker.cpp`

**职责**：
- 收 `PATH_READY` MQ → 读 `planner_path` SHM → `SetPath()`
- 读 `vehicle_state` SHM (100Hz) → `Compute()` → 写入 `control_cmd` SHM

**Pure Pursuit 算法流程**：
1. 在路径上找离车最近的点
2. 从最近点向前找前视距离 `Ld` 处的目标点
3. 目标点转换到车辆坐标系 → 计算横向偏移 `local_y`
4. 曲率 `κ = 2·local_y / Ld²` → 转向角 `δ = atan2(κ·轴距, 1)`
5. 曲率自适应速度 + P 控制器 → 加速度指令

**配置参数**（`run_tracker.cpp` 第 31-37 行，硬编码）：

| 参数 | 值 | 含义 |
|------|-----|------|
| lookahead_distance | 1.5 | 基础前视距离 (米) |
| target_velocity | 2.0 | 目标速度 (m/s) |
| max_steering | 0.6 | 最大转向角 (rad) |
| wheel_base | 2.0 | 轴距 (米) |
| goal_tolerance_xy | 0.3 | 终点判定距离 (米) |
| kp_velocity | 1.5 | 速度 P 增益 |

**Pure Pursuit 算法参数**（`pure_pursuit_tracker.cpp` 内修改）：

| 参数 | 位置 | 值 | 含义 |
|------|------|-----|------|
| 前视速度系数 | 第 77 行 `1.0 * abs(v)` | 1.0 | 速度越高前视越远 |
| 前视范围 | 第 78 行 `max(1.0, min(4.0, ld))` | [1.0, 4.0] | 前视距离上下限 |
| 转向速率限制 | 第 93 行 `max_rate = 3.0` | 3.0 rad/s | 防转向突变 |
| 弯道减速系数 | 第 103 行 `10.0 * abs_curvature` | 10.0 | 越大弯道越慢 |
| 加速度限制 | 第 108 行 `max(-2.0, min(2.0, ...))` | ±2.0 m/s² | 加减速上限 |

---

## 四、启动文件

| 文件 | 用途 | 启动的节点 |
|------|------|-----------|
| `src/Hybrid_A_Star/launch/full_system.launch` | **全系统启动**（日常使用） | roscore → map_server → Gazebo+spawn → bridge(延迟1s) → planner(2s) → tracker(2s) → RViz → TF静态发布 |
| `src/Hybrid_A_Star/launch/gazebo_simulation.launch` | 仅Gazebo+小车（调试模型用） | Gazebo + spawn_car + robot_state_publisher |
| `src/Hybrid_A_Star/launch/run_hybrid_a_star.launch` | 旧版单进程（不用） | 原始 run_hybrid_astar |

**启动命令**：
```bash
source devel/setup.bash
roslaunch hybrid_a_star full_system.launch
```

---

## 五、关键数据结构

### SHM 数据结构（定义在 `data_protocol.h`）

```
VehicleStateData:          PlannerPathData:           ControlCmdData:
  is_valid, sequence         is_valid, sequence         is_valid, sequence
  x, y, theta                path_size                  acceleration
  linear_velocity            path_points[50000*4]      steering_angle
  angular_velocity           (x,y,θ,v_ref 交错存储)     timestamp_ns
  timestamp_ns

GoalPoseData:              MapShmHeader:
  is_valid, sequence         is_valid, sequence
  x, y, theta                width, height
  timestamp_ns               resolution, origin_x/y
                             后面紧跟 uint8_t grid[width*height]
```

### 通用类型（定义在 `common/types.h`）

```
Pose2D {x, y, theta}
VehicleState {x, y, theta, linear_velocity, angular_velocity, timestamp}
ControlCommand {acceleration, steering_angle, timestamp}
PathPoint {x, y, theta, v_ref}
Path = vector<PathPoint>
MapData {grid, width, height, resolution, origin_x, origin_y}
PlannerConfig {...}   TrackerConfig {...}
```

---

## 六、目录速查

```
src/
├── common/                        # 公共库（头文件）
│   └── include/common/
│       ├── types.h                # 全部数据类型 + 配置结构体
│       ├── thread_pool.h          # 通用线程池（预留，未深度使用）
│       └── timer.h                # 高精度计时器
│
├── ipc/                           # IPC 通信层（头文件）
│   └── include/ipc/
│       ├── data_protocol.h        # ⭐ SHM/MQ 命名 + 数据结构定义
│       ├── data_bus.h             # ⭐ DataBusWriter/Reader 读写接口
│       ├── shared_memory.h        # shm_open/mmap 底层封装
│       └── message_queue.h        # POSIX mq 底层封装
│
├── gazebo_bridge/                 # 进程1: Gazebo桥接
│   └── app/
│       └── run_gazebo_bridge.cpp  # ⭐ 唯一文件，所有桥接逻辑
│
├── planner/                       # 进程2: 路径规划
│   ├── app/
│   │   └── run_planner.cpp        # ⭐ 入口 + 配置参数 + 主循环
│   ├── include/planner/
│   │   ├── i_planner.h            # 抽象接口（换算法只需实现此接口）
│   │   ├── hybrid_a_star_planner.h# 接口实现
│   │   ├── hybrid_a_star.h        # A* 搜索核心
│   │   ├── rs_path.h              # Reeds-Shepp 曲线
│   │   └── state_node.h           # 搜索节点
│   └── src/
│       ├── hybrid_a_star_planner.cpp
│       ├── hybrid_a_star.cpp      # 搜索主循环 + 碰撞检测 + 邻居生成
│       └── rs_path.cpp            # RS 曲线计算
│
├── tracker/                       # 进程3: 轨迹跟踪
│   ├── app/
│   │   └── run_tracker.cpp        # ⭐ 入口 + 配置参数 + 主循环
│   ├── include/tracker/
│   │   ├── i_tracker.h            # 抽象接口（换MPC只需实现此接口）
│   │   └── pure_pursuit_tracker.h # Pure Pursuit 实现
│   └── src/
│       └── pure_pursuit_tracker.cpp # ⭐ 跟踪算法（前视/曲率/速度控制）
│
├── config/                        # YAML 配置（预留，代码中尚未加载）
│   ├── planner_config.yaml
│   └── tracker_config.yaml
│
├── vehicle_model/                 # 车辆模型
│   ├── urdf/
│   │   └── car.urdf.xacro         # ⭐ 4轮小车 URDF + diff_drive 插件配置
│   └── worlds/
│       └── test.world             # Gazebo 世界（障碍物+走廊）
│
└── Hybrid_A_Star/                 # 原始 ROS 包（launch/rviz/maps 在此）
    ├── launch/
    │   ├── full_system.launch     # ⭐ 全系统启动
    │   └── gazebo_simulation.launch # Gazebo 单独启动
    ├── maps/
    │   ├── map_gazebo.yaml        # 地图元数据（origin/resolution/negate）
    │   └── map_gazebo.pgm         # 地图图像
    ├── rviz/
    │   └── display_hybrid_astar.rviz # RViz 布局
    └── vehicle_model/             # 实际使用的 URDF（launch引用此路径）
        ├── urdf/car.urdf.xacro
        └── worlds/test.world

scripts/
├── rebuild_all.sh                 # 一键构建+部署
└── generate_map.py                # 生成与 test.world 匹配的地图
```

---

## 七、常见修改速查

| 要做什么 | 去哪里改 |
|----------|----------|
| 改小车速度 | `run_tracker.cpp` 第 32 行 `target_velocity` |
| 改转向灵敏度 | `pure_pursuit_tracker.cpp` 第 77-78 行前视距离 |
| 改弯道减速强度 | `pure_pursuit_tracker.cpp` 第 103 行系数 |
| 改 Hybrid A* 搜索参数 | `run_planner.cpp` 第 30-39 行 |
| 改碰撞检测车辆尺寸 | `hybrid_a_star_planner.cpp` 第 42 行 `SetVehicleShape()` |
| 改 RS 捷径触发距离 | `run_planner.cpp` 第 39 行 `shot_distance` |
| 改 diff_drive 物理参数 | `car.urdf.xacro` 插件配置段 |
| 改地图 | 修改 `generate_map.py` → 运行 → 重启 |
| 改 SHM 段名/结构体 | `data_protocol.h` |
| 添加新的 IPC 通道 | `data_protocol.h` 加定义 → 进程中加 Writer/Reader |
| 换跟踪算法 (如 MPC) | 实现 `ITracker` 接口 → 改 `run_tracker.cpp` 实例化 |
| 换规划算法 | 实现 `IPlanner` 接口 → 改 `run_planner.cpp` 实例化 |
| 改启动顺序/延迟 | `full_system.launch` 中 `launch-prefix="bash -c 'sleep N; ...'"`
| 查看 RViz 路径线 | 确认 `/planned_path` topic，绿色线条 |

---

## 八、TF 坐标系

```
map ← world ← odom ← base_link ← (4个轮子)
  ↑       ↑       ↑           ↑
  static  static  Bridge TF   robot_state_publisher
  (0,0,0) (0,0,0) (Gazebo     (joint_states)
                   真实位姿)
```

- `world→map` 和 `world→odom` 是静态恒等变换
- `odom→base_link` 由 Bridge 从 Gazebo model_states 真实位姿发布（绕过了 diff_drive 里程计积分误差）
- RViz 固定帧设为 `world` 或 `map` 均可

---

## 九、构建命令

```bash
bash rebuild_all.sh          # 全量构建 (catkin + cmake) + 部署到 devel
source devel/setup.bash      # 设置 ROS 环境
roslaunch hybrid_a_star full_system.launch  # 启动全系统
```
