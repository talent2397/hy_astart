# Hybrid A* 系统重构方案

## 一、当前项目状态

### 1.1 已有功能
- ✅ Hybrid A* 路径搜索（含 Reeds-Shepp 曲线）
- ✅ 碰撞检测（矩形车辆模型）
- ✅ RViz 可视化（路径/搜索树/车辆姿态）
- ✅ 静态地图加载（map_server）
- ✅ 基础线程池框架（未深入使用）

### 1.2 缺失功能
- ❌ 运动控制/轨迹跟踪
- ❌ Gazebo 仿真集成
- ❌ 实时车辆状态反馈
- ❌ 多进程架构
- ❌ 进程间通信机制

---

## 二、目标架构总览

```
┌──────────────────────────────────────────────────────────────────────────┐
│                           Gazebo 仿真环境                                 │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐               │
│  │ 小车模型      │    │ 传感器(LiDAR)│    │ 障碍物世界    │               │
│  │ (URDF/SDF)   │    │ IMU/Camera   │    │              │               │
│  └──────┬───────┘    └──────┬───────┘    └──────────────┘               │
│         │                   │                                            │
│   /car/odom            /car/scan                                         │
│   /car/cmd_vel         /car/imu                                          │
└─────────┼───────────────────┼────────────────────────────────────────────┘
          │                   │
          ▼                   ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                        ROS Master + Topic Bus                             │
│   ROS 仅用于 Gazebo 接口层，核心数据走共享内存                            │
└─────────────────────────────────────────────────────────────────────────┘
          │                   │
          ▼                   ▼
┌────────────────────────┐  ┌──────────────────────────────────────┐
│    进程1: Planner       │  │     进程2: MPC Controller             │
│   (hybrid_astar_planner)│  │    (mpc_tracking_controller)          │
│                         │  │                                       │
│  ┌───────────────────┐  │  │  ┌────────────────────────────────┐  │
│  │ 主线程             │  │  │  │ 主线程                          │  │
│  │ - ROS 接口层       │  │  │  │ - ROS 接口层 (odom订阅)        │  │
│  │ - 地图接收         │  │  │  │ - 控制指令发布 (cmd_vel)       │  │
│  │ - 目标点接收       │  │  │  │ - 状态机管理                   │  │
│  └───────┬───────────┘  │  │  └──────────────┬─────────────────┘  │
│          │              │  │                 │                     │
│  ┌───────┴───────────┐  │  │  ┌──────────────┴─────────────────┐  │
│  │ 线程池 (N workers)│  │  │  │ 线程池 (M workers)              │  │
│  │ - 邻居节点生成     │  │  │  │ - MPC 优化求解                 │  │
│  │ - 碰撞检测        │  │  │  │ - 线性化/离散化                │  │
│  │ - 启发式计算      │  │  │  │ - 约束检查                     │  │
│  │ - RS曲线计算      │  │  │  │ - 参考轨迹插值                 │  │
│  └───────────────────┘  │  │  └────────────────────────────────┘  │
│                         │  │                                       │
│  ┌───────────────────┐  │  │  ┌────────────────────────────────┐  │
│  │ 定时器线程         │  │  │  │ 定时器线程 (100Hz)              │  │
│  │ - 心跳发布         │  │  │  │ - 控制循环触发                 │  │
│  │ - 统计信息         │  │  │  │ - 状态监控                     │  │
│  └───────────────────┘  │  │  └────────────────────────────────┘  │
│                         │  │                                       │
└──────────┬──────────────┘  └────────────┬─────────────────────────┘
           │                              │
           │    ┌──────────────────┐      │
           └────┤  共享内存 (SHM)  ├──────┘
                ├──────────────────┤
                │ /planner_path    │ ← 规划路径 (Planner → MPC)
                │ /vehicle_state   │ ← 车辆状态 (共享读取)
                │ /mpc_debug       │ ← MPC 调试数据
                │ /planner_cmd     │ ← 规划器控制命令
                └──────────────────┘
```

---

## 三、进程详细设计

### 3.1 进程1：Hybrid A* Planner (`hybrid_astar_planner`)

**职责**：全局路径规划，根据地图和目标点生成可行驶路径。

**线程划分**：

| 线程 | 频率/触发 | 职责 |
|------|-----------|------|
| 主线程 (Main) | 10Hz 循环 | ROS 接口、共享内存管理、控制流程编排 |
| 线程池 (Worker Pool) | 按需触发 | 邻居生成、碰撞检测、启发式计算、RS曲线 |
| 定时器线程 | 1Hz | 心跳发布、统计信息 |

**核心类设计**：
```cpp
// planner_core.h
class PlannerCore {
public:
    // 初始化：从共享内存或ROS参数加载配置
    bool Init(const PlannerConfig& config);

    // 设置地图（从共享内存）
    void SetMap(const OccupancyGridData& map);

    // 触发规划
    // start_pose: 当前车辆位姿
    // goal_pose:  目标位姿
    // 返回: 规划是否成功
    bool Plan(const Pose2D& start_pose, const Pose2D& goal_pose);

    // 获取规划结果
    const Path& GetPath() const;

    // 获取搜索树（用于可视化）
    const SearchTree& GetSearchTree() const;

private:
    std::unique_ptr<HybridAStar> astar_searcher_;
    std::unique_ptr<ThreadPool> thread_pool_;
    Path current_path_;
    // ...
};
```

**数据流**：
```
输入:
  - 地图: ROS /map 或共享内存
  - 起点: 共享内存 /vehicle_state
  - 终点: ROS /move_base_simple/goal

输出:
  - 路径: 共享内存 /planner_path
  - 可视化: ROS /searched_path, /searched_tree
```

---

### 3.2 进程2：MPC Controller (`mpc_tracking_controller`)

**职责**：实时轨迹跟踪，通过模型预测控制计算最优控制指令。

**线程划分**：

| 线程 | 频率 | 职责 |
|------|------|------|
| 主线程 (Main) | 100Hz | ROS 接口、共享内存读取、控制循环 |
| MPC 求解器线程 | 100Hz（与主循环同步）| QP 求解、线性化 |
| 状态估计线程 | 200Hz | 车辆状态估计、传感器数据融合 |
| 监控线程 | 10Hz | 跟踪误差统计、调试数据发布 |

**MPC 控制器核心设计**：
```cpp
// mpc_controller.h
class MPCController {
public:
    struct MP CConfig {
        int horizon_length = 20;         // 预测时域 N
        double dt = 0.05;                // 离散时间步长 (s)
        double max_steering = 0.6;       // 最大转向角 (rad)
        double max_velocity = 3.0;       // 最大速度 (m/s)
        double max_acceleration = 1.0;   // 最大加速度 (m/s²)
        // 权重矩阵
        Eigen::MatrixXd Q;  // 状态误差权重 (4x4) [x, y, theta, v]
        Eigen::MatrixXd R;  // 控制输入权重 (2x2) [a, delta]
    };

    bool Init(const MP CConfig& config);

    // 设置参考轨迹（从共享内存读取）
    void SetReferencePath(const Path& path);

    // 求解控制指令
    // current_state: [x, y, theta, v]
    // 返回: [acceleration, steering_angle]
    ControlCommand Solve(const VehicleState& current_state);

private:
    // 车辆运动学模型
    // 状态: [x, y, theta, v]
    // 控制: [a, delta]
    void KinematicModel(const State& x, const Control& u, State& x_next);

    // 参考轨迹插值
    // 找到路径上距离当前状态最近的点，向前取 N 个点
    void InterpolateReference(const VehicleState& current_state,
                              MatrixXd& ref_states,
                              MatrixXd& ref_controls);

    // QP 问题构建与求解
    bool SolveQP(const MatrixXd& A, const MatrixXd& B,
                 const MatrixXd& Q, const MatrixXd& R,
                 const VectorXd& x0, const MatrixXd& ref,
                 VectorXd& optimal_u);

    // 使用 qpOASES 或 OSQP 求解器
    std::unique_ptr<QPSolver> qp_solver_;
    MP CConfig config_;
    Path reference_path_;
};
```

**MPC 车辆运动学模型**（离散时间）：

```
状态: x = [x, y, θ, v]^T
控制: u = [a, δ]^T    (加速度, 前轮转角)

运动方程:
  x(k+1) = x(k) + v(k)*cos(θ(k))*dt
  y(k+1) = y(k) + v(k)*sin(θ(k))*dt
  θ(k+1) = θ(k) + v(k)*tan(δ(k))/L * dt
  v(k+1) = v(k) + a(k)*dt

线性化: 围绕参考轨迹进行一阶泰勒展开
  x̃(k+1) = A_k * x̃(k) + B_k * ũ(k)

其中 x̃ = x - x_ref, ũ = u - u_ref
```

**MPC 优化问题公式**：

```
min  Σ(k=0 to N-1) [ x̃(k)^T Q x̃(k) + ũ(k)^T R ũ(k) ]
     + x̃(N)^T Q_f x̃(N)

s.t.
  x̃(k+1) = A_k x̃(k) + B_k ũ(k)    (线性化动力学)
  x̃(0) = x_current - x_ref(0)       (初始状态)
  u_min ≤ u(k) ≤ u_max              (控制约束)
  Δu_min ≤ u(k) - u(k-1) ≤ Δu_max   (控制变化率约束)
  |δ(k)| ≤ δ_max                    (转向角约束)
  v_min ≤ v(k) ≤ v_max              (速度约束)
```

---

## 四、进程间通信（IPC）设计

### 4.1 共享内存 (Shared Memory)

用于高频大数据传输（路径数据、车辆状态）。

```cpp
// ipc/shared_memory.h

// 共享内存段名称定义
#define SHM_PLANNER_PATH    "/hybrid_astar_planner_path"
#define SHM_VEHICLE_STATE   "/hybrid_astar_vehicle_state"
#define SHM_MPC_DEBUG       "/hybrid_astar_mpc_debug"
#define SHM_PLANNER_CMD     "/hybrid_astar_planner_cmd"

// 路径数据（规划 → MPC）
struct PlannerPathData {
    std::atomic<bool> is_valid;       // 数据有效性标志
    std::atomic<uint64_t> timestamp;  // 时间戳 (ns)
    uint32_t path_size;               // 路径点数量
    // 路径点数组: [x, y, theta, v_ref] * path_size
    // 使用柔性数组或固定大小环形缓冲
    double path_points[MAX_PATH_POINTS * 4];
};

// 车辆状态（Gazebo → 所有进程）
struct VehicleStateData {
    std::atomic<bool> is_valid;
    std::atomic<uint64_t> timestamp;
    double x, y, theta;              // 位姿
    double vx, vy, omega;            // 速度
    double steering_angle;           // 当前转向角
    double linear_velocity;          // 线速度
};

// MPC 调试数据
struct MPC DebugData {
    std::atomic<bool> is_valid;
    std::atomic<uint64_t> timestamp;
    double tracking_error_x;
    double tracking_error_y;
    double tracking_error_theta;
    double solve_time_ms;
    double predicted_trajectory[MAX_HORIZON * 2]; // 预测轨迹
    double control_command[2];                     // [acc, delta]
};

// 规划器控制命令
struct PlannerCmdData {
    std::atomic<bool> is_valid;
    std::atomic<uint64_t> timestamp;
    enum Command { NONE = 0, REPLAN = 1, STOP = 2, PAUSE = 3 };
    Command cmd;
    double goal_x, goal_y, goal_theta;  // 重规划目标点
};
```

**同步机制**：
- 使用 `std::atomic` 标志 + `futex` 或 POSIX 信号量
- 写入端：写入数据 → `atomic_store(&is_valid, true)`
- 读取端：读取前检查 `is_valid`，读取后比对 `timestamp`

### 4.2 消息队列 (POSIX Message Queue)

用于低延迟事件通知。

```cpp
// ipc/message_queue.h

// 消息类型
enum class IPCMessageType : uint32_t {
    PATH_READY = 1,        // 规划完成，新路径可用
    REPLAN_REQUEST = 2,    // 请求重规划
    MPC_READY = 3,        // MPC 就绪
    PLANNER_READY = 4,    // Planner 就绪
    EMERGENCY_STOP = 5,   // 紧急停止
    STATUS_REPORT = 6,    // 状态报告
};

struct IPCMessage {
    IPCMessageType type;
    uint64_t timestamp;
    uint32_t payload_size;
    char payload[256];      // 可变长度负载
};
```

**消息队列名称**：
```cpp
#define MQ_PLANNER_TO_MPC  "/mq_planner_to_mpc"
#define MQ_MPC_TO_PLANNER  "/mq_mpc_to_planner"
```

### 4.3 Unix Domain Socket

用于配置和控制类通信（启动/停止/参数更新）。

```cpp
// ipc/control_socket.h

// Socket 路径
#define SOCK_PLANNER_CTRL  "/tmp/hybrid_astar_planner.sock"
#define SOCK_MPC_CTRL      "/tmp/hybrid_astar_mpc.sock"

// 控制协议（JSON 格式）
// 例: {"cmd": "set_param", "key": "max_velocity", "value": 2.0}
// 例: {"cmd": "shutdown"}
// 例: {"cmd": "get_status"}
```

### 4.4 IPC 总体数据流

```
时间敏感数据 (高频):
  Planner --[SHM]--> MPC (路径数据, ~1Hz 更新)
  Gazebo --[SHM]--> Planner, MPC (车辆状态, ~100Hz 更新)

事件通知 (低延迟):
  Planner --[MQ]--> MPC ("路径就绪" 通知)
  MPC    --[MQ]--> Planner ("重规划请求" 通知)

控制/配置 (低频):
  管理工具 --[Socket]--> Planner (配置更新)
  管理工具 --[Socket]--> MPC (配置更新)
```

---

## 五、目录结构设计

```
hybrid_a_star/
├── src/
│   ├── CMakeLists.txt                    # 顶层 CMake
│   │
│   ├── common/                           # 公共库
│   │   ├── CMakeLists.txt
│   │   ├── include/common/
│   │   │   ├── types.h                   # 通用类型定义
│   │   │   ├── math_utils.h              # 数学工具函数
│   │   │   ├── thread_pool.h             # 通用线程池
│   │   │   ├── timer.h                   # 高精度计时器
│   │   │   └── config_parser.h           # 配置解析
│   │   └── src/
│   │       ├── math_utils.cpp
│   │       ├── thread_pool.cpp
│   │       └── config_parser.cpp
│   │
│   ├── ipc/                              # 进程间通信库
│   │   ├── CMakeLists.txt
│   │   ├── include/ipc/
│   │   │   ├── shared_memory.h           # 共享内存封装
│   │   │   ├── message_queue.h           # 消息队列封装
│   │   │   ├── control_socket.h          # Unix Socket 封装
│   │   │   └── data_protocol.h           # 数据协议定义
│   │   └── src/
│   │       ├── shared_memory.cpp
│   │       ├── message_queue.cpp
│   │       └── control_socket.cpp
│   │
│   ├── planner/                          # 进程1: 路径规划器
│   │   ├── CMakeLists.txt
│   │   ├── include/planner/
│   │   │   ├── hybrid_a_star.h           # Hybrid A* 核心算法
│   │   │   ├── state_node.h              # 搜索状态节点
│   │   │   ├── rs_path.h                 # Reeds-Shepp 曲线
│   │   │   ├── collision_checker.h       # 碰撞检测
│   │   │   ├── planner_core.h            # 规划器核心（线程管理）
│   │   │   ├── costmap_interface.h       # 代价地图接口
│   │   │   └── planner_ipc.h             # 规划器 IPC 接口
│   │   ├── src/
│   │   │   ├── hybrid_a_star.cpp
│   │   │   ├── state_node.cpp
│   │   │   ├── rs_path.cpp
│   │   │   ├── collision_checker.cpp
│   │   │   ├── planner_core.cpp
│   │   │   ├── costmap_interface.cpp
│   │   │   └── planner_ipc.cpp
│   │   └── app/
│   │       └── run_planner.cpp            # 规划器主入口
│   │
│   ├── mpc_controller/                   # 进程2: MPC 控制器
│   │   ├── CMakeLists.txt
│   │   ├── include/mpc/
│   │   │   ├── mpc_controller.h          # MPC 控制核心
│   │   │   ├── kinematic_model.h         # 车辆运动学模型
│   │   │   ├── qp_solver.h               # QP 求解器接口
│   │   │   ├── reference_tracker.h       # 参考轨迹插值
│   │   │   ├── state_estimator.h         # 状态估计器
│   │   │   ├── control_interface.h       # 控制接口 (cmd_vel)
│   │   │   └── mpc_ipc.h                 # MPC IPC 接口
│   │   ├── src/
│   │   │   ├── mpc_controller.cpp
│   │   │   ├── kinematic_model.cpp
│   │   │   ├── qp_solver.cpp
│   │   │   ├── reference_tracker.cpp
│   │   │   ├── state_estimator.cpp
│   │   │   ├── control_interface.cpp
│   │   │   └── mpc_ipc.cpp
│   │   └── app/
│   │       └── run_mpc.cpp                # MPC 主入口
│   │
│   ├── gazebo_bridge/                    # Gazebo 仿真桥接
│   │   ├── CMakeLists.txt
│   │   ├── include/gazebo_bridge/
│   │   │   ├── vehicle_state_publisher.h  # 车辆状态发布（写入SHM）
│   │   │   └── control_subscriber.h       # 控制指令订阅（读取cmd_vel）
│   │   ├── src/
│   │   │   ├── vehicle_state_publisher.cpp
│   │   │   └── control_subscriber.cpp
│   │   └── app/
│   │       └── run_gazebo_bridge.cpp      # Gazebo桥接入口
│   │
│   ├── vehicle_model/                    # 车辆模型
│   │   ├── urdf/
│   │   │   └── car_model.urdf.xacro      # 车辆 URDF（含Gazebo插件）
│   │   ├── worlds/
│   │   │   └── hybrid_astar_world.world  # Gazebo 世界文件
│   │   └── config/
│   │       └── car_control.yaml          # 车辆控制参数
│   │
│   ├── launch/                           # 启动文件
│   │   ├── gazebo_simulation.launch      # Gazebo 仿真启动
│   │   ├── hybrid_astar_full.launch      # 完整系统启动
│   │   └── rviz_display.launch           # RViz 可视化启动
│   │
│   ├── config/                           # 全局配置
│   │   ├── planner_config.yaml           # 规划器参数
│   │   ├── mpc_config.yaml               # MPC 参数
│   │   └── ipc_config.yaml               # IPC 配置
│   │
│   └── scripts/                          # 工具脚本
│       ├── start_all.sh                  # 启动所有进程
│       ├── stop_all.sh                   # 停止所有进程
│       └── monitor.sh                    # 系统监控脚本
│
├── maps/                                 # 地图文件
│   ├── map.yaml
│   └── map.pgm
│
└── CMakeLists.txt                        # 顶层 CMake
```

---

## 六、关键技术实现方案

### 6.1 线程池设计

```cpp
// common/include/common/thread_pool.h
class ThreadPool {
public:
    explicit ThreadPool(size_t num_threads);

    // 提交任务，返回 future
    template<typename F, typename... Args>
    auto Submit(F&& f, Args&&... args)
        -> std::future<typename std::result_of<F(Args...)>::type>;

    // 批量并行执行
    template<typename Iterator, typename Func>
    void ParallelFor(Iterator begin, Iterator end, Func&& func);

    // 等待所有任务完成
    void WaitAll();

    // 获取统计信息
    struct Stats {
        size_t tasks_completed;
        size_t tasks_queued;
        double avg_execution_time_ms;
    };
    Stats GetStats() const;

private:
    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex queue_mutex_;
    std::condition_variable condition_;
    bool stop_;
};
```

### 6.2 规划器工作流程

```
1. 进程启动
   ├── 解析命令行参数 + YAML 配置文件
   ├── 初始化 ROS 节点 (planner_nh)
   ├── 打开共享内存 (SHM_PLANNER_PATH, SHM_VEHICLE_STATE)
   ├── 打开消息队列 (MQ_PLANNER_TO_MPC, MQ_MPC_TO_PLANNER)
   ├── 创建线程池 (N = CPU核心数 - 1)
   └── 进入主循环

2. 主循环 (10Hz)
   ├── 检查消息队列 (是否有重规划请求?)
   ├── 读取共享内存 (车辆当前状态)
   ├── 检查地图是否更新
   ├── 如果有新目标点:
   │   ├── 提交搜索任务到线程池
   │   │   ├── Worker 1-3: 邻居节点并行生成
   │   │   ├── Worker 4-6: 并行碰撞检测
   │   │   └── Worker 7:   启发式并行计算
   │   ├── 等待搜索完成
   │   ├── 路径后处理 (平滑/优化)
   │   ├── 写入共享内存 (规划路径)
   │   └── 发送消息队列 (PATH_READY)
   └── 发布可视化数据 (ROS topics)

3. 退出
   ├── 关闭共享内存
   ├── 关闭消息队列
   └── 线程池停止
```

### 6.3 MPC 控制器工作流程

```
1. 进程启动
   ├── 解析参数配置
   ├── 初始化 ROS 节点 (mpc_nh)
   ├── 打开共享内存 (读取路径 + 状态)
   ├── 打开消息队列
   ├── 初始化 QP 求解器 (OSQP)
   ├── 创建线程 (MPC求解线程, 状态估计线程)
   └── 进入主循环

2. 主循环 (100Hz)
   ├── 读取共享内存 (车辆状态 + 规划路径)
   ├── 状态估计 (滤波/融合)
   ├── 检查路径有效性
   ├── 触发 MPC 求解:
   │   ├── 参考轨迹插值 (找到最近点, 取N个前向点)
   │   ├── 系统线性化 (围绕参考轨迹)
   │   ├── 构建 QP 问题
   │   ├── 求解 QP (OSQP/qpOASES)
   │   └── 提取最优控制序列首元素
   ├── 发布 cmd_vel (或直接发布 /car/steering, /car/throttle)
   └── 写入 MPC 调试数据到共享内存

3. 紧急处理
   ├── 如果跟踪误差 > 阈值:
   │   ├── 发送重规划请求到消息队列
   │   └── 使用上一周期控制指令 (零阶保持)
   └── 如果连续丢帧 > N:
       └── 紧急制动 (v=0, delta=0)
```

### 6.4 状态估计器

```cpp
// mpc/include/mpc/state_estimator.h
class StateEstimator {
public:
    // 融合 odometry + IMU 数据
    VehicleState Estimate(const Odometry& odom, const ImuData& imu);

    // 扩展卡尔曼滤波
    void Predict(double dt);
    void Update(const Measurement& measurement);

    // 获取当前状态
    VehicleState GetState() const;

private:
    // EKF 状态: [x, y, theta, v, omega]
    Eigen::Matrix<double, 5, 1> state_;
    Eigen::Matrix<double, 5, 5> covariance_;

    // 过程噪声
    Eigen::Matrix<double, 5, 5> Q_;
    // 测量噪声
    Eigen::Matrix<double, 3, 3> R_;
};
```

### 6.5 进程管理

使用 systemd 或自定义守护进程管理生命周期：

```bash
# scripts/hybrid_astar_daemon.sh
# 守护进程：监控所有子进程，异常退出时自动重启

# 启动顺序
1. roscore
2. gazebo (带小车模型 + 世界)
3. gazebo_bridge (状态发布)
4. hybrid_astar_planner
5. mpc_tracking_controller
6. rviz (可视化)

# 停止顺序 (反向)
1. mpc_tracking_controller
2. hybrid_astar_planner
3. gazebo_bridge
4. gazebo
5. rviz
6. roscore
```

### 6.6 配置管理

所有配置使用 YAML 格式，支持运行时通过 Unix Socket 更新：

```yaml
# config/planner_config.yaml
planner:
  steering_angle: 15.0
  steering_angle_discrete_num: 1
  wheel_base: 2.0
  segment_length: 1.6
  segment_length_discrete_num: 8
  steering_penalty: 1.5
  reversing_penalty: 3.0
  steering_change_penalty: 2.0
  shot_distance: 5.0
  thread_pool_size: 7

# config/mpc_config.yaml
mpc:
  horizon_length: 20
  dt: 0.05
  max_steering: 0.6
  max_velocity: 3.0
  max_acceleration: 1.0
  Q_diag: [10.0, 10.0, 5.0, 1.0]   # x, y, theta, v 的权重
  R_diag: [0.1, 1.0]                 # a, delta 的权重
  tracking_tolerance_xy: 0.1
  tracking_tolerance_theta: 0.05
  thread_pool_size: 2

# config/ipc_config.yaml
ipc:
  shm_path_prefix: "/hybrid_astar"
  mq_max_messages: 100
  mq_max_message_size: 1024
  socket_path_prefix: "/tmp/hybrid_astar"
```

---

## 七、QP 求解器选择

| 求解器 | 优点 | 缺点 | 适用场景 |
|--------|------|------|----------|
| **OSQP** | BSD 协议, 轻量, 无外部依赖, C 实现 | 中小规模QP | 推荐：MPC 在线求解 |
| qpOASES | 活动集方法, 支持热启动 | LGPL, 较重 | 备选 |
| HPIPM | 高性能, 针对MPC优化 | BLASFEO依赖 | 追求极致性能时 |
| Eigen + 自定义 | 完全可控 | 开发量大 | 不推荐 |

**推荐使用 OSQP**：
- 轻量级 C 实现，易于集成
- 支持热启动（warm start），适合 MPC 迭代求解
- BSD 协议，无许可证风险
- 对中小规模 QP（N=20, 4状态+2控制）性能充足

---

## 八、实施路线图

### 阶段一：基础设施搭建（预计1周）
- [ ] 目录结构调整
- [ ] 公共库 (common)：线程池、数学工具、配置解析
- [ ] IPC 库：共享内存、消息队列、Socket 封装
- [ ] CMake 顶层配置

### 阶段二：Gazebo 仿真环境（预计1周）
- [ ] 小车 URDF/SDF 模型（含 Gazebo 物理插件）
- [ ] Gazebo 世界文件（障碍物环境）
- [ ] gazebo_bridge 节点（状态发布 + 控制订阅）
- [ ] 启动文件

### 阶段三：规划器重构（预计1周）
- [ ] 从原项目提取 Hybrid A* 核心代码
- [ ] 重构为线程池并行架构
- [ ] 集成 IPC 接口（路径写入共享内存）
- [ ] 独立可运行测试

### 阶段四：MPC 控制器实现（预计2周）
- [ ] 车辆运动学模型实现
- [ ] 参考轨迹插值器
- [ ] OSQP 集成 + QP 问题构建
- [ ] 状态估计器（EKF）
- [ ] 控制接口 (cmd_vel)
- [ ] 独立可运行测试

### 阶段五：系统集成与调试（预计1周）
- [ ] 全系统联调（Gazebo + Planner + MPC）
- [ ] 多种场景测试（直线、转弯、倒车、复杂环境）
- [ ] 性能优化（求解时间 < 5ms @100Hz）
- [ ] 稳定性测试

### 阶段六：优化与完善（按需）
- [ ] 动态障碍物处理
- [ ] 在线重规划
- [ ] 多车协同
- [ ] 性能调优

---

## 九、关键技术指标

| 指标 | 目标值 | 说明 |
|------|--------|------|
| 路径规划时间 | < 500ms | Hybrid A* 搜索 + RS 路径 |
| MPC 求解周期 | 10ms (100Hz) | 实时控制要求 |
| QP 求解时间 | < 3ms | OSQP 单次求解 |
| 共享内存延迟 | < 1μs | 同机内存映射 |
| 消息队列延迟 | < 100μs | POSIX MQ |
| 跟踪误差 (xy) | < 0.1m | 稳态跟踪 |
| 跟踪误差 (θ) | < 0.05 rad | 稳态跟踪 |
| 最大规划距离 | > 100m | 地图范围 |

---

## 十、风险与应对

| 风险 | 等级 | 应对措施 |
|------|------|----------|
| MPC 求解时间不满足实时要求 | 高 | 1) 减小预测时域 2) 使用热启动 3) 简化车辆模型 |
| 共享内存数据竞争 | 中 | 使用双缓冲 + atomic 标志位 |
| Gazebo 与真实车辆模型差异 | 中 | 保留模型参数配置接口，便于调参 |
| 进程异常退出导致共享内存泄漏 | 低 | 使用 RAII + 信号处理清理资源 |
| ROS1 线程安全问题 | 中 | 使用 ros::CallbackQueue 隔离 ROS 回调 |

---

## 十一、构建系统

```cmake
# 顶层 CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
project(hybrid_a_star_system)

# C++17: std::optional, if constexpr, structured bindings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 子项目
add_subdirectory(src/common)
add_subdirectory(src/ipc)
add_subdirectory(src/planner)
add_subdirectory(src/mpc_controller)
add_subdirectory(src/gazebo_bridge)

# 外部依赖
# - Eigen3: 线性代数
# - OSQP: QP 求解器
# - yaml-cpp: 配置文件解析
# - ROS (roscpp, nav_msgs, geometry_msgs, tf, gazebo_ros)
# - OpenMP (可选): 并行加速
```

---

## 十二、与原始项目的关系

- 原 `HybridAStar` 核心搜索算法保留，迁移到 `planner/` 模块
- 原 `RSPath` (Reeds-Shepp) 保留，迁移到 `planner/`
- 原 `ThreadPool` 升级为通用组件，放入 `common/`
- 原 `StateNode`, `type.h` 等基础类型迁移到 `common/`
- 原 ROS 接口层 (`HybridAStarFlow`) 拆分：
  - 规划相关 → `planner/planner_core`
  - 可视化部分保持

---

## 附录 A: 进程通信矩阵

| 数据 | 生产者 | 消费者 | 机制 | 频率 | 大小 |
|------|--------|--------|------|------|------|
| 规划路径 | Planner | MPC | 共享内存 + MQ通知 | ~1Hz | ~40KB |
| 车辆状态 | Gazebo Bridge | Planner, MPC | 共享内存 | 100Hz | ~64B |
| 控制命令 | MPC | Gazebo | ROS Topic | 100Hz | ~32B |
| MPC调试 | MPC | 监控工具 | 共享内存 | 100Hz | ~1KB |
| 规划命令 | MPC | Planner | 消息队列 | 按需 | ~64B |
| 地图 | ROS Map Server | Planner | ROS Topic | 启动时 | ~1MB |
| 目标点 | RViz | Planner | ROS Topic | 按需 | ~32B |
| 可视化 | Planner, MPC | RViz | ROS Topic | 10Hz | ~100KB |

## 附录 B: 信号处理

```
SIGTERM / SIGINT → 优雅退出 (保存状态, 清理IPC资源)
SIGUSR1         → Planner: 触发重规划
SIGUSR2         → MPC: 紧急制动
SIGALRM         → 看门狗定时器
```

## 附录 C: 日志系统

- 使用 glog (已在原项目中使用)
- 规划器日志: `~/.hybrid_astar/planner.log`
- MPC 日志: `~/.hybrid_astar/mpc.log`
- 系统日志: syslog
- 共享内存状态: 可通过 `/proc/shm` 或自定义工具查看
