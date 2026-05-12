/**
 * @file run_planner.cpp
 * @brief Hybrid A* 规划器进程入口
 *
 * 进程职责:
 *   1. 从 SHM 读取车辆状态、目标位姿、初始位姿、地图
 *   2. 调用 Hybrid A* 算法规划路径
 *   3. 将路径写入 SHM (planner_path)
 *   4. 通过 MQ 通知 Tracker: 新路径已就绪
 *
 * 主循环频率: ~10Hz (每次规划完成后等待至 100ms 周期)
 *
 * 数据流:
 *   SHM(vehicle_state, goal_pose, map_data) → Planner → SHM(planner_path) → MQ(PATH_READY) → Tracker
 */

#include "planner/hybrid_a_star_planner.h"
#include "ipc/data_bus.h"
#include "ipc/data_protocol.h"

#include <glog/logging.h>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>

/// 全局运行标志, 由信号处理函数设置为 false 触发优雅退出
static std::atomic<bool> g_running{true};

void SignalHandler(int sig) {
    LOG(INFO) << "Received signal " << sig << ", shutting down...";
    g_running = false;
}

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_logtostderr = true;

    signal(SIGTERM, SignalHandler);
    signal(SIGINT, SignalHandler);

    LOG(INFO) << "=== Hybrid A* Planner Process Starting ===";

    // ============================================================
    // 第 1 步: 初始化 Planner 核心
    // ============================================================
    PlannerConfig planner_cfg;
    // TODO: 从 YAML 文件加载配置 (目前硬编码)
    planner_cfg.steering_angle = 15.0;         // 最大转向角 15°
    planner_cfg.wheel_base = 2.0;              // 轴距 2m
    planner_cfg.segment_length = 1.6;          // 搜索步长 1.6m
    planner_cfg.segment_length_discrete_num = 8; // 每步细分 8 段 (碰撞检测粒度)
    planner_cfg.steering_penalty = 1.5;         // 转向惩罚
    planner_cfg.reversing_penalty = 3.0;        // 倒车惩罚
    planner_cfg.steering_change_penalty = 2.0;  // 转向变化惩罚
    planner_cfg.shot_distance = 5.0;            // RS 捷径触发距离 5m

    HybridAStarPlanner planner;
    if (!planner.Init(planner_cfg)) {
        LOG(FATAL) << "Failed to initialize planner";
        return 1;
    }

    // ============================================================
    // 第 2 步: 打开所有 IPC 通道 (SHM + MQ)
    // ============================================================
    DataBusReader<VehicleStateData> vehicle_reader;
    DataBusReader<GoalPoseData> goal_reader;
    DataBusReader<GoalPoseData> init_reader;
    DataBusReader<MapShmHeader> map_reader;
    DataBusWriter<PlannerPathData> path_writer;
    MessageQueue<IPCMessage> mq_planner_to_tracker;

    // 等待 vehicle_state SHM (Bridge 创建, ~100Hz 更新)
    LOG(INFO) << "Waiting for vehicle_state SHM...";
    while (g_running && !vehicle_reader.Open(SHM_VEHICLE_STATE)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 等待 goal_pose SHM (Bridge 创建, 用户设置目标点时更新)
    LOG(INFO) << "Waiting for goal_pose SHM...";
    while (g_running && !goal_reader.Open(SHM_GOAL_POSE)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 等待 initial_pose SHM (Bridge 创建, RViz 2D Pose Estimate 时更新)
    while (g_running && !init_reader.Open(SHM_INITIAL_POSE)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 等待并打开 map SHM
    // 第一步: 以 header 大小打开, 读取 width/height
    LOG(INFO) << "Waiting for map SHM...";
    while (g_running && !map_reader.Open(SHM_MAP_DATA, sizeof(MapShmHeader))) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    // 第二步: 等待地图数据就绪, 读取 header 获取实际尺寸
    MapShmHeader temp_header;
    while (g_running && !map_reader.ReadLatest(temp_header)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // 第三步: 关闭后以 header+grid 完整大小重新打开
    size_t map_total_size = sizeof(MapShmHeader) + temp_header.width * temp_header.height;
    map_reader.Close();
    while (g_running && !map_reader.Open(SHM_MAP_DATA, map_total_size)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    LOG(INFO) << "Map SHM opened: " << temp_header.width << "x" << temp_header.height
              << " total=" << map_total_size << " bytes";

    // 创建 planner_path SHM (Planner 是写入方)
    if (!path_writer.Open(SHM_PLANNER_PATH)) {
        LOG(FATAL) << "Failed to create planner_path SHM";
        return 1;
    }

    // 创建 PATH_READY 消息队列 (Planner → Tracker 通知)
    if (!mq_planner_to_tracker.Create(MQ_PATH_READY)) {
        LOG(FATAL) << "Failed to create PATH_READY MQ";
        return 1;
    }
    // 创建 REPLAN_REQUEST 消息队列 (预留: 外部请求重规划)
    MessageQueue<IPCMessage> mq_replan;
    if (!mq_replan.Create(MQ_REPLAN_REQUEST)) {
        LOG(FATAL) << "Failed to create REPLAN_REQUEST MQ";
        return 1;
    }

    LOG(INFO) << "IPC channels ready.";

    // ============================================================
    // 第 3 步: 加载地图 (静态数据, 只读一次)
    // ============================================================
    MapData map;
    {
        MapShmHeader map_header;
        // 阻塞等待地图数据写入
        while (g_running && !map_reader.ReadLatest(map_header)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        map.width = map_header.width;
        map.height = map_header.height;
        map.resolution = map_header.resolution;
        map.origin_x = map_header.origin_x;
        map.origin_y = map_header.origin_y;
        // 地图 grid 数据紧跟在 MapShmHeader 之后
        const uint8_t* raw_data = map_reader.GetRaw() + sizeof(MapShmHeader);
        map.data.assign(raw_data, raw_data + map.width * map.height);
        LOG(INFO) << "Map loaded into planner: " << map.width << "x" << map.height;
    }

    // ============================================================
    // 第 4 步: 主循环 (~10Hz)
    //
    // 循环流程:
    //   a. 读取车辆状态 (当前位置作为规划起点)
    //   b. 检查是否有 initial_pose (RViz 设置的起点, 优先于车辆当前位置)
    //   c. 检查是否有新目标点 (goal_pose)
    //   d. 执行 Hybrid A* 搜索
    //   e. 成功 → 写入 planner_path SHM → 发送 PATH_READY MQ → 清空目标
    //   f. 失败 → 警告, 清空目标, 等待新目标
    //   g. Reset() 清理搜索状态
    //   h. 控制频率 (补齐 100ms 周期)
    // ============================================================
    bool has_last_goal = false;
    GoalPoseData last_goal;

    while (g_running) {
        auto cycle_start = std::chrono::steady_clock::now();

        // 4a. 读取车辆状态 (非阻塞, 无新数据则 sleep 后重试)
        VehicleStateData vehicle;
        if (!vehicle_reader.ReadLatest(vehicle)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        // 4b. 检查是否有 RViz 设置的初始位姿 (作为起点覆盖)
        GoalPoseData init_pose;
        bool use_init_pose = init_reader.ReadLatest(init_pose);

        // 4c. 读取目标位姿
        GoalPoseData goal;
        bool has_new_goal = goal_reader.ReadLatest(goal);

        if (!has_new_goal && !has_last_goal) {
            // 无目标, 等待
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        // 检测新目标 (通过 sequence 变化)
        if (has_new_goal && (!has_last_goal ||
            goal.sequence != last_goal.sequence)) {
            last_goal = goal;
            has_last_goal = true;
            LOG(INFO) << "New goal accepted: (" << goal.x << ","
                      << goal.y << "," << goal.theta << ") seq="
                      << goal.sequence;
        }

        if (!has_last_goal) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        // 4d. 确定起点: initialpose 优先, 否则用车辆当前位置
        Pose2D start(vehicle.x, vehicle.y, vehicle.theta);
        if (use_init_pose) {
            start = Pose2D(init_pose.x, init_pose.y, init_pose.theta);
            LOG_FIRST_N(INFO, 1) << "Using initialpose as start: ("
                                 << start.x << "," << start.y << "," << start.theta << ")";
        }
        Pose2D end(last_goal.x, last_goal.y, last_goal.theta);
        Path planned_path;

        // 4e. 执行 Hybrid A* 搜索
        bool success = planner.Plan(start, end, map, planned_path);

        if (success) {
            // 4f. 将路径写入 SHM (交错数组格式: x0,y0,θ0,v0, x1,y1,θ1,v1, ...)
            PlannerPathData path_data;
            path_data.path_size = std::min(static_cast<uint32_t>(planned_path.size()),
                                            static_cast<uint32_t>(MAX_PATH_POINTS));
            path_data.timestamp_ns = static_cast<uint64_t>(
                std::chrono::steady_clock::now().time_since_epoch().count());

            for (size_t i = 0; i < path_data.path_size; ++i) {
                path_data.path_points[i * 4 + 0] = planned_path[i].x;
                path_data.path_points[i * 4 + 1] = planned_path[i].y;
                path_data.path_points[i * 4 + 2] = planned_path[i].theta;
                path_data.path_points[i * 4 + 3] = planned_path[i].v_ref;
            }

            path_writer.Write(path_data);

            // 4g. 通过 MQ 通知 Tracker: 新路径已就绪
            IPCMessage msg;
            msg.type = IPCMessageType::PATH_READY;
            msg.timestamp_ns = path_data.timestamp_ns;
            mq_planner_to_tracker.Send(msg);

            LOG(INFO) << "Path published: " << path_data.path_size << " points to SHM";

            // 清空目标标志, 等待下一次目标
            has_last_goal = false;
        } else {
            LOG(WARNING) << "Planning failed from (" << start.x << "," << start.y
                         << ") to (" << end.x << "," << end.y << ")";
            has_last_goal = false;
        }

        // 清理本次搜索状态 (openset/closeset/搜索树), 保留地图
        planner.Reset();

        // 4h. 控制频率 ~10Hz (100ms 周期)
        auto cycle_end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(cycle_end - cycle_start);
        if (elapsed.count() < 100) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100) - elapsed);
        }
    }

    LOG(INFO) << "Planner process exiting.";
    return 0;
}
