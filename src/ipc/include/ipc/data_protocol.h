#ifndef HYBRID_ASTAR_IPC_DATA_PROTOCOL_H
#define HYBRID_ASTAR_IPC_DATA_PROTOCOL_H

#include <cstdint>
#include <cstddef>

// ============================================================
// 共享内存命名约定 (POSIX shm_open 路径)
//
// 所有 SHM 段名以 "/hy_astar_" 为前缀, 位于 /dev/shm/ 下
// 每个 SHM 段对应一个数据结构, 单写者单读者 (或单写者多读者)
// ============================================================

/// 车辆状态: Bridge 写入 (来自 Gazebo model_states), Planner/Tracker 读取
#define SHM_VEHICLE_STATE   "/hy_astar_vehicle_state"

/// 规划路径: Planner 写入, Tracker 和 Bridge (用于 RViz 可视化) 读取
#define SHM_PLANNER_PATH    "/hy_astar_planner_path"

/// 控制指令: Tracker 写入 (加速度 + 转向角), Bridge 读取并转换为 cmd_vel
#define SHM_CONTROL_CMD     "/hy_astar_control_cmd"

/// 目标位姿: Bridge 写入 (来自 RViz /move_base_simple/goal), Planner 读取
#define SHM_GOAL_POSE       "/hy_astar_goal_pose"

/// 初始位姿: Bridge 写入 (来自 RViz /initialpose), Planner 读取作为规划起点
#define SHM_INITIAL_POSE    "/hy_astar_initial_pose"

/// 地图数据: Bridge 写入 (来自 ROS /map topic), Planner 读取
#define SHM_MAP_DATA        "/hy_astar_map_data"

// ============================================================
// 消息队列命名约定 (POSIX mq_open 路径)
//
// 用于低频事件通知, 延迟 < 100μs
// ============================================================

/// 路径就绪通知: Planner → Tracker, 告知新路径已写入 SHM
#define MQ_PATH_READY       "/hy_astar_path_ready"

/// 重规划请求: (预留) Tracker/外部 → Planner, 请求重新规划
#define MQ_REPLAN_REQUEST   "/hy_astar_replan_request"

/// 紧急停止: (预留) 任意进程 → Tracker, 触发紧急制动
#define MQ_EMERGENCY_STOP   "/hy_astar_emergency_stop"

// ============================================================
// 最大路径点数量 (共享内存中固定大小)
//
// 50000 点 × 4 个 double (x,y,θ,v_ref) × 8 字节 = 1.6 MB
// 对于 100m 范围、0.1m 分辨率足够
// ============================================================
#define MAX_PATH_POINTS 50000

// ============================================================
// 共享内存数据结构
//
// 设计要点:
//   1. 全部为 POD 类型 (trivially copyable), 可用 memcpy
//   2. 不使用 std::atomic (不可 trivially copy), 改用 uint8_t + uint64_t 手动同步
//   3. 同步协议: Writer 先设 is_valid=0 → memcpy → sequence++ → is_valid=1
//      Reader 检测 sequence 变化来判断是否有新数据 (无锁, 零拷贝)
// ============================================================

/**
 * @brief 车辆状态 (SHM_VEHICLE_STATE)
 *
 * Bridge 订阅 /gazebo/model_states, 以 ~100Hz 写入
 * Planner 和 Tracker 以各自频率读取
 */
struct VehicleStateData {
    uint8_t is_valid = 0;       // 0 = 写入中/无效, 1 = 数据有效可读
    uint64_t sequence = 0;      // 递增序列号, Reader 通过检测变化来判断新数据

    double x = 0.0;             // 世界 X (米) — 来自 Gazebo 真实位姿
    double y = 0.0;             // 世界 Y (米)
    double theta = 0.0;         // 偏航角 (弧度)
    double linear_velocity = 0.0;   // 前进方向速度 (m/s), 已投影到车辆坐标系
    double angular_velocity = 0.0;  // 角速度 (rad/s)
    double steering_angle = 0.0;    // 当前转向角 (弧度)
    uint64_t timestamp_ns = 0;      // 数据时间戳 (纳秒)
};

/**
 * @brief 规划路径 (SHM_PLANNER_PATH)
 *
 * Planner 搜索完成后写入, Tracker 和 Bridge 读取
 * 路径点以交错数组存储: [x0, y0, θ0, v0, x1, y1, θ1, v1, ...]
 * 这样布局利于内存连续性和缓存友好
 */
struct PlannerPathData {
    uint8_t is_valid = 0;
    uint64_t sequence = 0;

    uint32_t path_size = 0;                         // 实际路径点数
    uint64_t timestamp_ns = 0;                      // 规划完成时间
    double path_points[MAX_PATH_POINTS * 4];        // 路径点交错数组 (x,y,θ,v_ref)
};

/**
 * @brief 控制指令 (SHM_CONTROL_CMD)
 *
 * Tracker 以 100Hz 写入 (加速度 + 转向角)
 * Bridge 读取后积分加速度得到速度, 结合转向角计算 cmd_vel
 */
struct ControlCmdData {
    uint8_t is_valid = 0;
    uint64_t sequence = 0;

    double acceleration = 0.0;      // 加速度 (m/s²)
    double steering_angle = 0.0;    // 前轮转向角 (弧度)
    uint64_t timestamp_ns = 0;
};

/**
 * @brief 目标/初始位姿 (SHM_GOAL_POSE / SHM_INITIAL_POSE)
 *
 * 两个 SHM 段共用此结构体
 * Bridge 接收 ROS topic (/move_base_simple/goal, /initialpose) 后写入
 */
struct GoalPoseData {
    uint8_t is_valid = 0;
    uint64_t sequence = 0;

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    uint64_t timestamp_ns = 0;
};

/**
 * @brief 地图 SHM 头部 (SHM_MAP_DATA)
 *
 * 可变长度数据: Header 后紧跟 uint8_t grid[width * height]
 * SHM 段总大小 = sizeof(MapShmHeader) + width * height * sizeof(uint8_t)
 *
 * 读取流程: 先以 sizeof(header) 打开 → 读 width/height → 计算总大小 → 重新以完整大小打开
 */
struct MapShmHeader {
    uint8_t is_valid = 0;
    uint64_t sequence = 0;

    int width = 0;              // 栅格列数
    int height = 0;             // 栅格行数
    double resolution = 0.1;    // 每格边长 (米)
    double origin_x = 0.0;      // 地图原点世界 X
    double origin_y = 0.0;      // 地图原点世界 Y
    uint64_t timestamp_ns = 0;
    // 数据紧跟在后: uint8_t data[width * height]
    // 访问方式: reinterpret_cast<uint8_t*>(header + 1)
};

// ============================================================
// 消息队列消息类型
// ============================================================

/// 消息类型枚举 — 同一 MQ 可收发多种类型的消息
enum class IPCMessageType : uint32_t {
    PATH_READY = 1,         // 新路径已就绪 (Planner → Tracker)
    REPLAN_REQUEST = 2,     // 请求重规划 (预留)
    EMERGENCY_STOP = 3,     // 紧急停止 (预留)
    PLANNER_READY = 4,      // Planner 就绪通知 (预留)
    TRACKER_READY = 5,      // Tracker 就绪通知 (预留)
    GOAL_REACHED = 6,       // 已到达终点 (预留)
};

/// 消息队列中传输的消息体
struct IPCMessage {
    IPCMessageType type;        // 消息类型
    uint64_t timestamp_ns;      // 发送时间戳
    uint32_t payload_size;      // 有效载荷字节数
    char payload[128];          // 可变载荷 (最多 128 字节)

    IPCMessage() : type(IPCMessageType::PATH_READY),
                   timestamp_ns(0), payload_size(0) {
        payload[0] = '\0';
    }
};

#endif // HYBRID_ASTAR_IPC_DATA_PROTOCOL_H
