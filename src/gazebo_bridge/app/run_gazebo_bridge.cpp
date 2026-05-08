#include "ipc/data_bus.h"
#include "ipc/data_protocol.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

#include <glog/logging.h>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

/**
 * @brief Gazebo Bridge 进程
 *
 * 职责：在 ROS Topic 和共享内存数据总线之间做双向转换。
 *   ROS → SHM: /car/odom, /map, /move_base_simple/goal
 *   SHM → ROS: /car/cmd_vel
 *
 * 这是唯一依赖 ROS 的进程。换仿真平台只需重写这个进程。
 */

static std::atomic<bool> g_running{true};

void SignalHandler(int sig) {
    LOG(INFO) << "Bridge received signal " << sig;
    g_running = false;
}

static DataBusWriter<VehicleStateData> g_vehicle_writer;
static DataBusWriter<GoalPoseData> g_goal_writer;
static DataBusWriter<MapShmHeader> g_map_shm_writer;
static DataBusReader<ControlCmdData> g_control_reader;
static ros::Publisher* g_cmd_vel_pub = nullptr;
static int g_map_fd = -1;  // mmap fd for map data
static uint8_t* g_map_mmap = nullptr;
static size_t g_map_mmap_size = 0;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    VehicleStateData state;
    state.x = msg->pose.pose.position.x;
    state.y = msg->pose.pose.position.y;
    state.theta = tf::getYaw(msg->pose.pose.orientation);
    state.linear_velocity = msg->twist.twist.linear.x;
    state.angular_velocity = msg->twist.twist.angular.z;
    state.timestamp_ns = msg->header.stamp.toNSec();
    g_vehicle_writer.Write(state);
}

void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    GoalPoseData goal;
    goal.x = msg->pose.position.x;
    goal.y = msg->pose.position.y;
    goal.theta = tf::getYaw(msg->pose.orientation);
    goal.timestamp_ns = msg->header.stamp.toNSec();
    g_goal_writer.Write(goal);
    LOG(INFO) << "New goal received: (" << goal.x << "," << goal.y << "," << goal.theta << ")";
}

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

    // 写入 header + data 到共享内存
    // 由于 MapShmHeader + data 的结构，使用 mmap 直接写入
    // 简化实现：重新创建 SHM 并写入完整数据
    if (g_map_mmap && g_map_mmap_size >= total_size) {
        std::memcpy(g_map_mmap, &header, sizeof(MapShmHeader));
        std::memcpy(g_map_mmap + sizeof(MapShmHeader), msg->data.data(), data_size);
        // 标记有效
        reinterpret_cast<MapShmHeader*>(g_map_mmap)->is_valid = 1;
        reinterpret_cast<MapShmHeader*>(g_map_mmap)->sequence++;
    }

    LOG_FIRST_N(INFO, 1) << "Map received: " << header.width << "x" << header.height;
}

void ControlLoop() {
    ros::Rate rate(100);  // 100Hz
    while (g_running && ros::ok()) {
        ControlCmdData cmd;
        if (g_control_reader.ReadLatest(cmd)) {
            geometry_msgs::Twist twist;
            twist.linear.x = cmd.acceleration;   // 简化：加速度直接映射到线速度
            // 实际应该积分：v += a*dt, 但简化先直接发布
            twist.angular.z = cmd.steering_angle;
            g_cmd_vel_pub->publish(twist);
        }
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    signal(SIGTERM, SignalHandler);
    signal(SIGINT, SignalHandler);

    LOG(INFO) << "=== Gazebo Bridge Process Starting ===";

    // ---- ROS 初始化 ----
    ros::init(argc, argv, "gazebo_bridge", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    // ---- 创建共享内存通道 ----
    if (!g_vehicle_writer.Open(SHM_VEHICLE_STATE)) {
        LOG(FATAL) << "Failed to create vehicle_state SHM";
        return 1;
    }
    if (!g_goal_writer.Open(SHM_GOAL_POSE)) {
        LOG(FATAL) << "Failed to create goal_pose SHM";
        return 1;
    }

    // 创建地图共享内存（带数据区）
    // 使用 posix shm 直接创建足够大的区域
    {
        size_t max_map_size = sizeof(MapShmHeader) + 4000 * 4000;  // up to 4000x4000
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
        std::memset(g_map_mmap, 0, max_map_size);
    }

    // ---- ROS Subscribers ----
    ros::Subscriber odom_sub = nh.subscribe("/car/odom", 1, OdomCallback);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, GoalCallback);
    ros::Subscriber map_sub = nh.subscribe("/map", 1, MapCallback);

    // ---- ROS Publisher ----
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/car/cmd_vel", 1);
    g_cmd_vel_pub = &cmd_vel_pub;

    // ---- 打开控制指令 SHM (reader) ----
    // 等待 Tracker 创建
    LOG(INFO) << "Waiting for control_cmd SHM...";
    while (g_running && ros::ok() && !g_control_reader.Open(SHM_CONTROL_CMD)) {
        ros::Duration(0.5).sleep();
    }

    LOG(INFO) << "Gazebo Bridge ready. All IPC channels connected.";

    // ---- 控制发布循环 ----
    ControlLoop();

    // ---- 清理 ----
    if (g_map_mmap) munmap(g_map_mmap, g_map_mmap_size);
    if (g_map_fd >= 0) close(g_map_fd);

    // 清理所有 SHM
    shm_unlink(SHM_VEHICLE_STATE);
    shm_unlink(SHM_GOAL_POSE);
    shm_unlink(SHM_MAP_DATA);
    mq_unlink(MQ_PATH_READY);
    mq_unlink(MQ_REPLAN_REQUEST);

    ros::shutdown();
    LOG(INFO) << "Gazebo Bridge exiting.";
    return 0;
}
