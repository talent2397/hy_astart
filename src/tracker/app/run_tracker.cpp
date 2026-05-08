#include "tracker/pure_pursuit_tracker.h"
#include "ipc/data_bus.h"
#include "ipc/data_protocol.h"

#include <glog/logging.h>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>

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

    LOG(INFO) << "=== MPC / Pure Pursuit Tracker Process Starting ===";

    // ---- 1. 初始化 Tracker ----
    TrackerConfig tracker_cfg;
    // TODO: 从 YAML 文件加载配置
    tracker_cfg.lookahead_distance = 3.0;
    tracker_cfg.target_velocity = 1.0;
    tracker_cfg.max_steering = 0.6;
    tracker_cfg.wheel_base = 2.0;
    tracker_cfg.goal_tolerance_xy = 0.2;
    tracker_cfg.kp_velocity = 0.5;

    PurePursuitTracker tracker;
    if (!tracker.Init(tracker_cfg)) {
        LOG(FATAL) << "Failed to initialize tracker";
        return 1;
    }

    // ---- 2. 打开共享内存和消息队列 ----
    DataBusReader<VehicleStateData> vehicle_reader;
    DataBusReader<PlannerPathData> path_reader;
    DataBusWriter<ControlCmdData> control_writer;
    MessageQueue<IPCMessage> mq_path_ready;
    MessageQueue<IPCMessage> mq_replan;

    // 等待 vehicle_state SHM
    LOG(INFO) << "Waiting for vehicle_state SHM...";
    while (g_running && !vehicle_reader.Open(SHM_VEHICLE_STATE)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 等待 planner_path SHM
    LOG(INFO) << "Waiting for planner_path SHM...";
    while (g_running && !path_reader.Open(SHM_PLANNER_PATH)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 创建 control_cmd SHM
    if (!control_writer.Open(SHM_CONTROL_CMD)) {
        LOG(FATAL) << "Failed to create control_cmd SHM";
        return 1;
    }

    // 打开消息队列
    while (g_running && !mq_path_ready.Open(MQ_PATH_READY))
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

    while (g_running && !mq_replan.Open(MQ_REPLAN_REQUEST))
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

    LOG(INFO) << "IPC channels ready. Control loop starting...";

    // ---- 3. 主控制循环 (100Hz) ----
    auto last_path_update = std::chrono::steady_clock::now();
    [[maybe_unused]] bool has_path = false;

    while (g_running) {
        auto cycle_start = std::chrono::steady_clock::now();

        // 3a. 检查是否有新路径（非阻塞）
        IPCMessage msg;
        if (mq_path_ready.TryReceive(msg)) {
            if (msg.type == IPCMessageType::PATH_READY) {
                // 读取新路径
                PlannerPathData path_data;
                if (path_reader.ReadLatest(path_data)) {
                    Path path;
                    path.reserve(path_data.path_size);
                    for (uint32_t i = 0; i < path_data.path_size; ++i) {
                        PathPoint pt;
                        pt.x = path_data.path_points[i * 4 + 0];
                        pt.y = path_data.path_points[i * 4 + 1];
                        pt.theta = path_data.path_points[i * 4 + 2];
                        pt.v_ref = path_data.path_points[i * 4 + 3];
                        path.push_back(pt);
                    }
                    tracker.SetPath(path);
                    has_path = true;
                    last_path_update = std::chrono::steady_clock::now();
                    LOG(INFO) << "Tracker received new path: " << path.size() << " points";
                }
            }
        }

        // 3b. 读取车辆状态
        VehicleStateData vehicle;
        if (!vehicle_reader.ReadLatest(vehicle)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // 3c. 计算控制指令
        VehicleState state;
        state.x = vehicle.x;
        state.y = vehicle.y;
        state.theta = vehicle.theta;
        state.linear_velocity = vehicle.linear_velocity;
        state.angular_velocity = vehicle.angular_velocity;
        state.timestamp = vehicle.timestamp_ns;

        ControlCommand cmd = tracker.Compute(state);

        // 3d. 检查是否到达目标
        if (tracker.IsGoalReached(state)) {
            LOG(INFO) << "Goal reached! Stopping.";
            cmd.acceleration = -0.5;  // 减速
            cmd.steering_angle = 0.0;
        }

        // 3e. 写入控制指令到共享内存
        ControlCmdData cmd_data;
        cmd_data.acceleration = cmd.acceleration;
        cmd_data.steering_angle = cmd.steering_angle;
        cmd_data.timestamp_ns = static_cast<uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count());
        control_writer.Write(cmd_data);

        // 4. 维持 100Hz 频率
        auto cycle_end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start);
        auto target = std::chrono::microseconds(10000);  // 10ms = 100Hz
        if (elapsed < target) {
            std::this_thread::sleep_for(target - elapsed);
        } else {
            // 超过周期，警告
            static int overrun_count = 0;
            if (++overrun_count % 100 == 0) {
                LOG(WARNING) << "Control loop overrun: " << elapsed.count() << "us";
            }
        }
    }

    LOG(INFO) << "Tracker process exiting.";
    return 0;
}
