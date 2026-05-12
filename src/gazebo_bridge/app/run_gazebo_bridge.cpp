/**
 * @file run_gazebo_bridge.cpp
 * @brief Gazebo Bridge 进程 — ROS Topic 与共享内存 (SHM) 之间的双向桥接
 *
 * 这是系统中唯一与 ROS 直接交互的进程, 其他进程 (Planner, Tracker)
 * 完全通过 SHM 通信, 不依赖 ROS。
 *
 * 数据方向:
 *   ROS → SHM (Bridge 订阅, 写入 SHM):
 *     /gazebo/model_states      → SHM_VEHICLE_STATE   (100Hz, 车辆真实位姿+速度)
 *     /map                       → SHM_MAP_DATA        (启动时1次, 占用栅格地图)
 *     /move_base_simple/goal     → SHM_GOAL_POSE       (按需, RViz 目标点)
 *     /initialpose               → SHM_INITIAL_POSE    (按需, RViz 初始位姿)
 *
 *   SHM → ROS (Bridge 读取 SHM, 发布到 ROS):
 *     SHM_CONTROL_CMD   → /cmd_vel            (100Hz, 加速度积分+转向角→twist)
 *     SHM_PLANNER_PATH  → /planned_path       (按需, 路径可视化给 RViz)
 *
 * TF 发布:
 *   Bridge 发布 odom → base_link (使用 Gazebo 真实位姿, 绕过 diff_drive 里程计)
 *
 * 关键设计决策:
 *   - 使用 /gazebo/model_states (真实位姿) 而非 /odom (轮子积分),
 *     避免里程计累积误差和初始偏移 (Bug #18/#19 修复)
 *   - cmd_vel 每帧无条件发布 (不依赖 ReadLatest 结果),
 *     无新指令时使用上一帧的转向角和加速度维持 (Bug #21a 修复)
 *   - 路径可视化轮询重试 Open (Planner 晚启动时 SHM 尚未创建)
 */

#include "ipc/data_bus.h"
#include "ipc/data_protocol.h"

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <glog/logging.h>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

static std::atomic<bool> g_running{true};

void SignalHandler(int sig) {
    LOG(INFO) << "Bridge received signal " << sig;
    g_running = false;
}

// ============================================================
// 全局 IPC 读写器 (进程级单例)
// ============================================================

static DataBusWriter<VehicleStateData> g_vehicle_writer;     // ROS → SHM: 车辆状态
static DataBusWriter<GoalPoseData> g_goal_writer;            // ROS → SHM: 目标位姿
static DataBusWriter<GoalPoseData> g_init_writer;            // ROS → SHM: 初始位姿
static DataBusWriter<MapShmHeader> g_map_shm_writer;         // ROS → SHM: 地图
static DataBusReader<ControlCmdData> g_control_reader;       // SHM → ROS: 控制指令
static DataBusReader<PlannerPathData> g_path_reader;         // SHM → ROS: 规划路径(可视化)
static ros::Publisher* g_cmd_vel_pub = nullptr;              // 发布 /cmd_vel
static ros::Publisher* g_path_pub = nullptr;                 // 发布 /planned_path (RViz)
static ros::ServiceClient* g_set_model_client = nullptr;    // Gazebo set_model_state 服务
static tf::TransformBroadcaster* g_tf_broadcaster = nullptr; // 发布 odom→base_link
static int g_map_fd = -1;                                    // 地图 SHM 文件描述符
static uint8_t* g_map_mmap = nullptr;                        // 地图 SHM mmap 地址
static size_t g_map_mmap_size = 0;                           // 地图 SHM 映射大小
static uint64_t g_last_path_seq = 0;                         // 上次发布路径的 sequence
static constexpr double kWheelBase = 2.0;                    // 自行车模型轴距 (米)

// ============================================================
// ROS 回调函数: 将 ROS Topic 数据写入 SHM
// ============================================================

/**
 * @brief Gazebo 模型状态回调 (替代旧 /odom 回调)
 *
 * 订阅 /gazebo/model_states (100Hz), 遍历模型名找到 "car",
 * 用 Gazebo 真实位姿 (非轮子积分) 更新 SHM 和 TF。
 *
 * 关键修复 (Bug #23):
 *   twist.linear.x/y 是 Gazebo 世界坐标系的速度分量,
 *   必须投影到车辆前进方向才能给 Tracker 使用:
 *     forward_speed = vx*cos(θ) + vy*sin(θ)
 *
 *   例如: 车朝北 (θ=π/2), vx=0, vy=1 → forward_speed = 0*0 + 1*1 = 1 m/s ✓
 */
void GazeboModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "car") {
            const auto& pose = msg->pose[i];
            const auto& twist = msg->twist[i];

            // ---- 写入 SHM vehicle_state (Gazebo 真实位姿) ----
            VehicleStateData state;
            state.x = pose.position.x;
            state.y = pose.position.y;
            state.theta = tf::getYaw(pose.orientation);
            // Bug #23 修复: 世界坐标系速度投影到车辆前进方向
            double cos_yaw = std::cos(state.theta);
            double sin_yaw = std::sin(state.theta);
            state.linear_velocity = twist.linear.x * cos_yaw + twist.linear.y * sin_yaw;
            state.angular_velocity = twist.angular.z;
            state.timestamp_ns = ros::Time::now().toNSec();
            g_vehicle_writer.Write(state);

            // ---- 发布 odom → base_link TF (Gazebo 真实位姿) ----
            // 绕过 diff_drive 的积分里程计, 消除坐标漂移 (Bug #18/#19 修复)
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, 0.0));
            transform.setRotation(tf::Quaternion(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w));
            g_tf_broadcaster->sendTransform(tf::StampedTransform(
                transform, ros::Time::now(), "odom", "base_link"));
            return;
        }
    }
}

/**
 * @brief 目标点回调 (RViz "2D Nav Goal")
 *
 * 将用户在 RViz 中设置的导航目标点写入 SHM, Planner 读取后触发规划。
 */
void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    GoalPoseData goal;
    goal.x = msg->pose.position.x;
    goal.y = msg->pose.position.y;
    goal.theta = tf::getYaw(msg->pose.orientation);
    goal.timestamp_ns = msg->header.stamp.toNSec();
    g_goal_writer.Write(goal);
    LOG(INFO) << "New goal received: (" << goal.x << "," << goal.y << "," << goal.theta << ")";
}

/**
 * @brief 初始位姿回调 (RViz "2D Pose Estimate")
 *
 * 1. 将初始位姿写入 SHM → Planner 可将其作为规划起点 (而非当前车辆位置)
 * 2. 调用 Gazebo set_model_state 服务将小车传送到该位姿
 *
 * Bug #11 修复: 之前只写 SHM, 小车物理位置未变,
 * 导致 RViz 显示的位置与实际小车位置不一致。
 */
void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    GoalPoseData init;
    init.x = msg->pose.pose.position.x;
    init.y = msg->pose.pose.position.y;
    init.theta = tf::getYaw(msg->pose.pose.orientation);
    init.timestamp_ns = msg->header.stamp.toNSec();
    g_init_writer.Write(init);

    // 传送 Gazebo 小车到 RViz 指定的位姿
    gazebo_msgs::SetModelState srv;
    srv.request.model_state.model_name = "car";
    srv.request.model_state.pose = msg->pose.pose;
    srv.request.model_state.reference_frame = "world";
    if (g_set_model_client && g_set_model_client->call(srv)) {
        LOG(INFO) << "Car teleported to: (" << init.x << "," << init.y << "," << init.theta << ")";
    } else {
        LOG(WARNING) << "Failed to teleport car via set_model_state";
    }
}

/**
 * @brief 地图回调 (ROS /map topic → SHM)
 *
 * 从 map_server 接收占用栅格地图, 写入可变长度 SHM 段:
 *   [MapShmHeader][uint8_t grid[width*height]]
 *
 * 注意: 地图 SHM 在 main() 中手动创建 (因为大小不固定, 不能用模板),
 * 写入时直接用 memcpy + 手动更新 is_valid/sequence。
 */
void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    MapShmHeader header;
    header.width = msg->info.width;
    header.height = msg->info.height;
    header.resolution = msg->info.resolution;
    header.origin_x = msg->info.origin.position.x;
    header.origin_y = msg->info.origin.position.y;
    header.timestamp_ns = ros::Time::now().toNSec();

    size_t data_size = header.width * header.height * sizeof(uint8_t);
    size_t total_size = sizeof(MapShmHeader) + data_size;

    // 直接 memcpy 到 mmap 区域 (比通过 DataBusWriter 更高效, 因为大小不固定)
    if (g_map_mmap && g_map_mmap_size >= total_size) {
        std::memcpy(g_map_mmap, &header, sizeof(MapShmHeader));
        std::memcpy(g_map_mmap + sizeof(MapShmHeader), msg->data.data(), data_size);
        // 手动更新同步字段
        reinterpret_cast<MapShmHeader*>(g_map_mmap)->is_valid = 1;
        reinterpret_cast<MapShmHeader*>(g_map_mmap)->sequence++;
    }

    LOG_FIRST_N(INFO, 1) << "Map received: " << header.width << "x" << header.height;
}

// ============================================================
// 控制循环 (100Hz) — SHM → ROS 方向
// ============================================================

/**
 * @brief 主控制循环: 读取 SHM 控制指令, 转换为 /cmd_vel 发布
 *
 * 关键逻辑:
 *   1. 轮询重试 Open (Planner/Tracker 可能晚于 Bridge 启动)
 *   2. ReadLatest 成功 → 积分加速度, 保存 last_steering / last_accel
 *   3. ReadLatest 失败 → 使用保存的 last_steering / last_accel 维持 (防丢帧)
 *      Bug #21a 修复: 之前没收到新指令时强制 angular_z=0, 车无法转弯
 *   4. 每帧无条件发布 cmd_vel (用 target_vel + last_steering)
 *   5. 路径可视化独立于控制指令 (PlannerPath SHM 可能尚未创建, 轮询重试)
 */
void ControlLoop() {
    ros::Rate rate(100);          // 100Hz
    double dt = 0.01;             // 时间步长 (秒)
    double target_vel = 0.0;      // 当前目标速度 (加速度积分结果)
    double last_steering = 0.0;   // 上一帧转向角 (防丢帧, Bug #21a 修复)
    double last_accel = 0.0;      // 上一帧加速度 (防丢帧, Bug #22 修复)

    // 尝试打开 SHM (可能晚于 Writer 创建, 轮询重试)
    bool control_ready = g_control_reader.Open(SHM_CONTROL_CMD);
    bool path_reader_ready = g_path_reader.Open(SHM_PLANNER_PATH);

    while (g_running && ros::ok()) {
        // ---- 轮询重试打开 SHM ----
        if (!control_ready) {
            control_ready = g_control_reader.Open(SHM_CONTROL_CMD);
            if (control_ready) {
                LOG(INFO) << "ControlCmd SHM connected!";
            }
        }
        if (!path_reader_ready) {
            path_reader_ready = g_path_reader.Open(SHM_PLANNER_PATH);
            if (path_reader_ready) {
                LOG(INFO) << "PlannerPath SHM connected!";
            }
        }

        // ---- 读取 Tracker 控制指令 ----
        ControlCmdData cmd;
        if (control_ready && g_control_reader.ReadLatest(cmd)) {
            // 收到新指令 → 积分加速度更新速度
            target_vel += cmd.acceleration * dt;
            target_vel = std::max(0.0, std::min(3.0, target_vel));  // 限幅 [0, 3.0] m/s
            last_steering = cmd.steering_angle;
            last_accel = cmd.acceleration;
        } else {
            // 未收到新指令 → 用上一次的加速度维持速度 (而非盲目减速)
            // Bug #22 修复: 之前用 target_vel -= 0.5*dt 衰减 → 车跑一会就停
            target_vel += last_accel * dt;
            target_vel = std::max(0.0, std::min(3.0, target_vel));
        }

        // ---- 发布 cmd_vel (每帧无条件执行) ----
        // 自行车模型: angular_z = v * tan(steering) / wheelbase
        {
            double angular_z = target_vel * std::tan(last_steering) / kWheelBase;
            angular_z = std::max(-1.0, std::min(1.0, angular_z));  // 角速度限幅 ±1.0 rad/s
            geometry_msgs::Twist twist;
            twist.linear.x = target_vel;
            twist.angular.z = angular_z;
            g_cmd_vel_pub->publish(twist);
        }

        // ---- 路径可视化 (独立于控制指令, 检测新路径时发布) ----
        // 直接读 SHM 指针 (零拷贝), 比较 sequence 判断是否有新路径
        {
            const PlannerPathData* path_ptr = g_path_reader.Get();
            if (path_ptr && path_ptr->is_valid && path_ptr->sequence != g_last_path_seq && path_ptr->path_size > 0) {
                g_last_path_seq = path_ptr->sequence;
                // 将交错数组格式转换为 ROS nav_msgs::Path
                nav_msgs::Path ros_path;
                ros_path.header.frame_id = "map";
                ros_path.header.stamp = ros::Time::now();
                ros_path.poses.reserve(path_ptr->path_size);
                for (uint32_t i = 0; i < path_ptr->path_size; ++i) {
                    geometry_msgs::PoseStamped pose;
                    pose.header.frame_id = "map";
                    pose.pose.position.x = path_ptr->path_points[i * 4 + 0];
                    pose.pose.position.y = path_ptr->path_points[i * 4 + 1];
                    pose.pose.position.z = 0;
                    double theta = path_ptr->path_points[i * 4 + 2];
                    pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
                    ros_path.poses.push_back(pose);
                }
                g_path_pub->publish(ros_path);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}

// ============================================================
// 主函数: 初始化 ROS + SHM, 进入控制循环
// ============================================================

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    signal(SIGTERM, SignalHandler);
    signal(SIGINT, SignalHandler);

    LOG(INFO) << "=== Gazebo Bridge Process Starting ===";

    // ROS 初始化 (不使用默认信号处理, 已有自定义 handler)
    ros::init(argc, argv, "gazebo_bridge", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    // ============================================================
    // 创建 SHM 写入通道 (Bridge 是以下通道的 Writer)
    // ============================================================
    if (!g_vehicle_writer.Open(SHM_VEHICLE_STATE)) {
        LOG(FATAL) << "Failed to create vehicle_state SHM";
        return 1;
    }
    if (!g_goal_writer.Open(SHM_GOAL_POSE)) {
        LOG(FATAL) << "Failed to create goal_pose SHM";
        return 1;
    }
    if (!g_init_writer.Open(SHM_INITIAL_POSE)) {
        LOG(FATAL) << "Failed to create initial_pose SHM";
        return 1;
    }

    // ============================================================
    // 创建地图 SHM (手动, 因为大小不固定)
    //
    // 预分配 4000×4000 格的地图空间 (约 16MB), 足够覆盖大多数地图。
    // 实际使用只映射 MapShmHeader + width*height 字节。
    // ============================================================
    {
        size_t max_map_size = sizeof(MapShmHeader) + 4000 * 4000;
        int fd = shm_open(SHM_MAP_DATA, O_CREAT | O_RDWR, 0666);
        if (fd < 0) {
            perror("shm_open map");
            LOG(FATAL) << "Failed to create map SHM";
            return 1;
        }
        if (ftruncate(fd, max_map_size) < 0) { perror("ftruncate map"); }
        g_map_mmap = static_cast<uint8_t*>(mmap(nullptr, max_map_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
        if (g_map_mmap == MAP_FAILED) {
            perror("mmap map");
            return 1;
        }
        g_map_fd = fd;
        g_map_mmap_size = max_map_size;
        std::memset(g_map_mmap, 0, max_map_size);  // 初始化为全零
    }

    // ============================================================
    // TF 广播器: 发布 odom → base_link (使用 Gazebo 真实位姿)
    //
    // TF 链路: map → world (static) → odom (static) → base_link (Gazebo 真实位姿, Bridge 发布)
    // Bug #18/#19 修复: 不用 diff_drive 的 TF, 改用 Bridge 自己发布真实位姿
    // ============================================================
    tf::TransformBroadcaster tf_broadcaster;
    g_tf_broadcaster = &tf_broadcaster;

    // Gazebo 传送服务 (用于 /initialpose 回调)
    ros::ServiceClient set_model_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    g_set_model_client = &set_model_client;

    // ============================================================
    // ROS Subscribers (ROS → SHM)
    // ============================================================
    ros::Subscriber model_sub = nh.subscribe("/gazebo/model_states", 1, GazeboModelStatesCallback);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, GoalCallback);
    ros::Subscriber init_sub = nh.subscribe("/initialpose", 1, InitPoseCallback);
    ros::Subscriber map_sub = nh.subscribe("/map", 1, MapCallback);

    // ============================================================
    // ROS Publishers (SHM → ROS)
    // ============================================================
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    g_cmd_vel_pub = &cmd_vel_pub;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/planned_path", 1);
    g_path_pub = &path_pub;

    LOG(INFO) << "Gazebo Bridge ready. Entering control loop...";

    // ============================================================
    // 进入 100Hz 控制循环 (阻塞)
    // ============================================================
    ControlLoop();

    // ============================================================
    // 清理资源
    // ============================================================
    if (g_map_mmap) munmap(g_map_mmap, g_map_mmap_size);
    if (g_map_fd >= 0) close(g_map_fd);

    shm_unlink(SHM_VEHICLE_STATE);
    shm_unlink(SHM_GOAL_POSE);
    shm_unlink(SHM_INITIAL_POSE);
    shm_unlink(SHM_MAP_DATA);
    mq_unlink(MQ_PATH_READY);
    mq_unlink(MQ_REPLAN_REQUEST);

    ros::shutdown();
    LOG(INFO) << "Gazebo Bridge exiting.";
    return 0;
}
