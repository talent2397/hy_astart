#ifndef HYBRID_ASTAR_IPC_DATA_PROTOCOL_H
#define HYBRID_ASTAR_IPC_DATA_PROTOCOL_H

#include <cstdint>
#include <cstddef>

// ============================================================
// 共享内存命名约定
// ============================================================
#define SHM_VEHICLE_STATE   "/hy_astar_vehicle_state"
#define SHM_PLANNER_PATH    "/hy_astar_planner_path"
#define SHM_CONTROL_CMD     "/hy_astar_control_cmd"
#define SHM_GOAL_POSE       "/hy_astar_goal_pose"
#define SHM_INITIAL_POSE    "/hy_astar_initial_pose"
#define SHM_MAP_DATA        "/hy_astar_map_data"

// 消息队列命名约定
#define MQ_PATH_READY       "/hy_astar_path_ready"
#define MQ_REPLAN_REQUEST   "/hy_astar_replan_request"
#define MQ_EMERGENCY_STOP   "/hy_astar_emergency_stop"

// ============================================================
// 最大路径点数量（共享内存中固定大小）
// ============================================================
#define MAX_PATH_POINTS 50000

// ============================================================
// 共享内存数据结构（POD 类型，可 trivially copyable）
// 注意：不使用 std::atomic（不可拷贝），用双缓冲 + sequence 保证同步
// ============================================================

struct VehicleStateData {
    uint8_t is_valid = 0;    // 0=invalid, 1=valid
    uint64_t sequence = 0;

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;
    double steering_angle = 0.0;
    uint64_t timestamp_ns = 0;
};

struct PlannerPathData {
    uint8_t is_valid = 0;
    uint64_t sequence = 0;

    uint32_t path_size = 0;
    uint64_t timestamp_ns = 0;
    double path_points[MAX_PATH_POINTS * 4];
};

struct ControlCmdData {
    uint8_t is_valid = 0;
    uint64_t sequence = 0;

    double acceleration = 0.0;
    double steering_angle = 0.0;
    uint64_t timestamp_ns = 0;
};

struct GoalPoseData {
    uint8_t is_valid = 0;
    uint64_t sequence = 0;

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    uint64_t timestamp_ns = 0;
};

struct MapShmHeader {
    uint8_t is_valid = 0;
    uint64_t sequence = 0;

    int width = 0;
    int height = 0;
    double resolution = 0.1;
    double origin_x = 0.0;
    double origin_y = 0.0;
    uint64_t timestamp_ns = 0;
    // 数据紧跟在后: uint8_t data[width * height]
};

// ============================================================
// 消息队列消息类型
// ============================================================

enum class IPCMessageType : uint32_t {
    PATH_READY = 1,
    REPLAN_REQUEST = 2,
    EMERGENCY_STOP = 3,
    PLANNER_READY = 4,
    TRACKER_READY = 5,
    GOAL_REACHED = 6,
};

struct IPCMessage {
    IPCMessageType type;
    uint64_t timestamp_ns;
    uint32_t payload_size;
    char payload[128];

    IPCMessage() : type(IPCMessageType::PATH_READY),
                   timestamp_ns(0), payload_size(0) {
        payload[0] = '\0';
    }
};

#endif // HYBRID_ASTAR_IPC_DATA_PROTOCOL_H
