/**
 * @file run_tracker.cpp
 * @brief 轨迹跟踪器进程入口 (当前使用 Pure Pursuit, 后续可切换为 MPC)
 *
 * 进程职责:
 *   1. 从 MQ 接收 PATH_READY 消息 → 读 SHM planner_path → SetPath()
 *   2. 从 SHM 读取 vehicle_state (100Hz)
 *   3. Compute() 计算控制指令 (加速度 + 转向角)
 *   4. 写入 SHM control_cmd → Bridge 转换为 cmd_vel 发给 Gazebo
 *   5. IsGoalReached() 检测是否到达终点 → 到达后减速停车
 *
 * 主循环频率: 100Hz (10ms 周期)
 *
 * 数据流:
 *   MQ(PATH_READY) + SHM(planner_path, vehicle_state) → Tracker → SHM(control_cmd) → Bridge → Gazebo
 */

#include "tracker/pure_pursuit_tracker.h"
#include "ipc/data_bus.h"
#include "ipc/data_protocol.h"

#include <glog/logging.h>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>

/// 全局运行标志, 信号处理函数设置为 false 触发优雅退出
static std::atomic<bool> g_running{true};

void SignalHandler(int sig)
{
    LOG(INFO) << "Received signal " << sig << ", shutting down...";
    g_running = false;
}

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_logtostderr = true;

    signal(SIGTERM, SignalHandler);
    signal(SIGINT, SignalHandler);

    LOG(INFO) << "=== MPC / Pure Pursuit Tracker Process Starting ===";

    // ============================================================
    // 第 1 步: 初始化 Tracker
    // ============================================================
    TrackerConfig tracker_cfg;
    // TODO: 从 YAML 文件加载配置 (目前硬编码)
    tracker_cfg.lookahead_distance = 1.5;   // 基础前视距离 1.5m
    tracker_cfg.target_velocity = 2.0;      // 目标速度 2.0 m/s
    tracker_cfg.max_steering = 0.6;         // 最大转向角 ~34°
    tracker_cfg.wheel_base = 2.0;           // 轴距 2m
    tracker_cfg.goal_tolerance_xy = 0.3;    // 终点判定 0.3m
    tracker_cfg.kp_velocity = 1.5;          // 速度 P 增益

    PurePursuitTracker tracker;
    if (!tracker.Init(tracker_cfg))
    {
        LOG(FATAL) << "Failed to initialize tracker";
        return 1;
    }

    // ============================================================
    // 第 2 步: 打开所有 IPC 通道 (SHM + MQ)
    // ============================================================
    DataBusReader<VehicleStateData> vehicle_reader;
    DataBusReader<PlannerPathData> path_reader;
    DataBusWriter<ControlCmdData> control_writer;
    MessageQueue<IPCMessage> mq_path_ready;
    MessageQueue<IPCMessage> mq_replan;

    // 等待 vehicle_state SHM (Bridge 创建, 100Hz 更新)
    LOG(INFO) << "Waiting for vehicle_state SHM...";
    while (g_running && !vehicle_reader.Open(SHM_VEHICLE_STATE))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 等待 planner_path SHM (Planner 创建, 路径更新时写入)
    while (g_running && !path_reader.Open(SHM_PLANNER_PATH))
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 创建 control_cmd SHM (Tracker 是写入方)
    if (!control_writer.Open(SHM_CONTROL_CMD))
    {
        LOG(FATAL) << "Failed to create control_cmd SHM";
        return 1;
    }

    // 打开 PATH_READY 消息队列 (Planner → Tracker 通知)
    while (g_running && !mq_path_ready.Open(MQ_PATH_READY))
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 打开 REPLAN_REQUEST 消息队列 (预留)
    while (g_running && !mq_replan.Open(MQ_REPLAN_REQUEST))
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

    LOG(INFO) << "IPC channels ready. Control loop starting...";

    // ============================================================
    // 第 3 步: 主控制循环 (100Hz)
    //
    // 循环流程:
    //   a. 非阻塞检查 MQ: 收到 PATH_READY → 读 SHM path → SetPath()
    //   b. 读取车辆状态 (SHM vehicle_state)
    //   c. Compute() 计算控制指令
    //   d. IsGoalReached() → 到达则减速停车
    //   e. 写入 control_cmd SHM
    //   f. 控制频率 (维持 10ms 周期)
    // ============================================================
    auto last_path_update = std::chrono::steady_clock::now();
    [[maybe_unused]] bool has_path = false;

    while (g_running)
    {
        auto cycle_start = std::chrono::steady_clock::now();

        // 3a. 非阻塞检查是否有新路径通知
        IPCMessage msg;
        if (mq_path_ready.TryReceive(msg))
        {
            if (msg.type == IPCMessageType::PATH_READY)
            {
                // 收到通知后从 SHM 读取完整的路径数据
                PlannerPathData path_data;
                if (path_reader.ReadLatest(path_data))
                {
                    // 将 SHM 交错数组格式转换为 Path 类型
                    Path path;
                    path.reserve(path_data.path_size);
                    for (uint32_t i = 0; i < path_data.path_size; ++i)
                    {
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

        // 3b. 读取车辆状态 (非阻塞, 无新数据则短暂 sleep 后重试)
        VehicleStateData vehicle;
        if (!vehicle_reader.ReadLatest(vehicle))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // 3c. 将 SHM 格式转换为通用 VehicleState, 然后调用跟踪算法
        VehicleState state;
        state.x = vehicle.x;
        state.y = vehicle.y;
        state.theta = vehicle.theta;
        state.linear_velocity = vehicle.linear_velocity;
        state.angular_velocity = vehicle.angular_velocity;
        state.timestamp = vehicle.timestamp_ns;

        ControlCommand cmd = tracker.Compute(state);

        // 3d. 检查是否到达终点: 到达或无路径时减速到零
        if (tracker.IsGoalReached(state))
        {
            if (tracker.HasPath())
            {
                LOG_FIRST_N(INFO, 1) << "Goal reached! Stopping.";
            }
            // 直接速度抵消: 加速度 = -当前速度 (一帧内停止)
            cmd.acceleration = -1.0 * state.linear_velocity;
            cmd.steering_angle = 0.0;
        }

        // 3e. 将控制指令写入 SHM (Bridge 读取后转换为 cmd_vel)
        ControlCmdData cmd_data;
        cmd_data.acceleration = cmd.acceleration;
        cmd_data.steering_angle = cmd.steering_angle;
        cmd_data.timestamp_ns = static_cast<uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count());
        control_writer.Write(cmd_data);

        // 3f. 维持 100Hz 频率 (10ms 周期)
        auto cycle_end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start);
        auto target = std::chrono::microseconds(10000);  // 10ms = 100Hz
        if (elapsed < target)
        {
            std::this_thread::sleep_for(target - elapsed);
        }
        else
        {
            // 超时警告 (每 500 次警告一次, 避免刷屏)
            static int overrun_count = 0;
            if (++overrun_count % 500 == 0)
            {
                LOG(WARNING) << "Control loop overrun: " << elapsed.count() << "us";
            }
        }
    }

    LOG(INFO) << "Tracker process exiting.";
    return 0;
}
