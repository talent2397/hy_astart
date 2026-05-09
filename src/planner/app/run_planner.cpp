#include "planner/hybrid_a_star_planner.h"
#include "ipc/data_bus.h"
#include "ipc/data_protocol.h"

#include <glog/logging.h>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>

// 全局标志：程序是否运行
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

    // ---- 1. 初始化 Planner ----
    PlannerConfig planner_cfg;
    // TODO: 从 YAML 文件加载配置
    planner_cfg.steering_angle = 15.0;
    planner_cfg.wheel_base = 2.0;
    planner_cfg.segment_length = 1.6;
    planner_cfg.segment_length_discrete_num = 8;
    planner_cfg.steering_penalty = 1.5;
    planner_cfg.reversing_penalty = 3.0;
    planner_cfg.steering_change_penalty = 2.0;
    planner_cfg.shot_distance = 5.0;

    HybridAStarPlanner planner;
    if (!planner.Init(planner_cfg)) {
        LOG(FATAL) << "Failed to initialize planner";
        return 1;
    }

    // ---- 2. 打开共享内存和消息队列 ----
    DataBusReader<VehicleStateData> vehicle_reader;
    DataBusReader<GoalPoseData> goal_reader;
    DataBusReader<GoalPoseData> init_reader;
    DataBusReader<MapShmHeader> map_reader;
    DataBusWriter<PlannerPathData> path_writer;
    MessageQueue<IPCMessage> mq_planner_to_tracker;

    // 等待 vehicle_state SHM 创建
    LOG(INFO) << "Waiting for vehicle_state SHM...";
    while (g_running && !vehicle_reader.Open(SHM_VEHICLE_STATE)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 等待 goal_pose SHM
    LOG(INFO) << "Waiting for goal_pose SHM...";
    while (g_running && !goal_reader.Open(SHM_GOAL_POSE)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 等待 initial_pose SHM
    while (g_running && !init_reader.Open(SHM_INITIAL_POSE)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 等待并打开 map SHM（需要先读取 header 获得 width/height 才能知道完整大小）
    LOG(INFO) << "Waiting for map SHM...";
    while (g_running && !map_reader.Open(SHM_MAP_DATA, sizeof(MapShmHeader))) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    // 读取 header 获取地图尺寸，然后重新打开完整 SHM
    MapShmHeader temp_header;
    while (g_running && !map_reader.ReadLatest(temp_header)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    size_t map_total_size = sizeof(MapShmHeader) + temp_header.width * temp_header.height;
    map_reader.Close();  // 关闭旧的，重新以完整大小打开
    while (g_running && !map_reader.Open(SHM_MAP_DATA, map_total_size)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    LOG(INFO) << "Map SHM opened: " << temp_header.width << "x" << temp_header.height
              << " total=" << map_total_size << " bytes";

    // 创建路径 SHM
    if (!path_writer.Open(SHM_PLANNER_PATH)) {
        LOG(FATAL) << "Failed to create planner_path SHM";
        return 1;
    }

    // 创建消息队列（PATH_READY 用于通知 Tracker；REPLAN_REQUEST 供外部请求重规划）
    if (!mq_planner_to_tracker.Create(MQ_PATH_READY)) {
        LOG(FATAL) << "Failed to create PATH_READY MQ";
        return 1;
    }
    MessageQueue<IPCMessage> mq_replan;
    if (!mq_replan.Create(MQ_REPLAN_REQUEST)) {
        LOG(FATAL) << "Failed to create REPLAN_REQUEST MQ";
        return 1;
    }

    LOG(INFO) << "IPC channels ready.";

    // ---- 3. 主循环前：加载地图（静态数据，只读一次）----
    MapData map;
    {
        MapShmHeader map_header;
        // 阻塞等待地图数据
        while (g_running && !map_reader.ReadLatest(map_header)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        map.width = map_header.width;
        map.height = map_header.height;
        map.resolution = map_header.resolution;
        map.origin_x = map_header.origin_x;
        map.origin_y = map_header.origin_y;
        const uint8_t* raw_data = map_reader.GetRaw() + sizeof(MapShmHeader);
        map.data.assign(raw_data, raw_data + map.width * map.height);
        LOG(INFO) << "Map loaded into planner: " << map.width << "x" << map.height;
    }

    // ---- 4. 主循环 (10Hz) ----
    bool has_last_goal = false;
    GoalPoseData last_goal;

    while (g_running) {
        auto cycle_start = std::chrono::steady_clock::now();

        // 4a. 读取车辆状态
        VehicleStateData vehicle;
        if (!vehicle_reader.ReadLatest(vehicle)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        // 4b. 检查是否有 RViz 设置的初始位姿（作为起点覆盖）
        GoalPoseData init_pose;
        bool use_init_pose = init_reader.ReadLatest(init_pose);

        // 4c. 读取目标位姿
        GoalPoseData goal;
        bool has_new_goal = goal_reader.ReadLatest(goal);

        if (!has_new_goal && !has_last_goal) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

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

        // 3d. 执行规划
        Pose2D start(vehicle.x, vehicle.y, vehicle.theta);
        if (use_init_pose) {
            start = Pose2D(init_pose.x, init_pose.y, init_pose.theta);
            LOG_FIRST_N(INFO, 1) << "Using initialpose as start: ("
                                 << start.x << "," << start.y << "," << start.theta << ")";
        }
        Pose2D end(last_goal.x, last_goal.y, last_goal.theta);
        Path planned_path;

        bool success = planner.Plan(start, end, map, planned_path);

        if (success) {
            // 3e. 写入路径到共享内存
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

            // 3f. 通知 Tracker
            IPCMessage msg;
            msg.type = IPCMessageType::PATH_READY;
            msg.timestamp_ns = path_data.timestamp_ns;
            mq_planner_to_tracker.Send(msg);

            LOG(INFO) << "Path published: " << path_data.path_size << " points to SHM";

            // 清空目标，等待下一次
            has_last_goal = false;
        } else {
            LOG(WARNING) << "Planning failed from (" << start.x << "," << start.y
                         << ") to (" << end.x << "," << end.y << ")";
            has_last_goal = false;
        }

        planner.Reset();

        // 控制频率
        auto cycle_end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(cycle_end - cycle_start);
        if (elapsed.count() < 100) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100) - elapsed);
        }
    }

    LOG(INFO) << "Planner process exiting.";
    return 0;
}
