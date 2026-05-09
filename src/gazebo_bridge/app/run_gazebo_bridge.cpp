#include "ipc/data_bus.h"
#include "ipc/data_protocol.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/SetModelState.h>
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
 *   ROS → SHM: /odom, /map, /move_base_simple/goal, /initialpose
 *   SHM → ROS: /cmd_vel, /planned_path
 */

static std::atomic<bool> g_running{true};

void SignalHandler(int sig) {
    LOG(INFO) << "Bridge received signal " << sig;
    g_running = false;
}

static DataBusWriter<VehicleStateData> g_vehicle_writer;
static DataBusWriter<GoalPoseData> g_goal_writer;
static DataBusWriter<GoalPoseData> g_init_writer;
static DataBusWriter<MapShmHeader> g_map_shm_writer;
static DataBusReader<ControlCmdData> g_control_reader;
static DataBusReader<PlannerPathData> g_path_reader;
static ros::Publisher* g_cmd_vel_pub = nullptr;
static ros::Publisher* g_path_pub = nullptr;
static ros::ServiceClient* g_set_model_client = nullptr;
static int g_map_fd = -1;
static uint8_t* g_map_mmap = nullptr;
static size_t g_map_mmap_size = 0;
static uint64_t g_last_path_seq = 0;
static constexpr double kWheelBase = 2.0;

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

void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    GoalPoseData init;
    init.x = msg->pose.pose.position.x;
    init.y = msg->pose.pose.position.y;
    init.theta = tf::getYaw(msg->pose.pose.orientation);
    init.timestamp_ns = msg->header.stamp.toNSec();
    g_init_writer.Write(init);

    // 传送 Gazebo 中小车到 RViz 指定的初始位姿
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

    if (g_map_mmap && g_map_mmap_size >= total_size) {
        std::memcpy(g_map_mmap, &header, sizeof(MapShmHeader));
        std::memcpy(g_map_mmap + sizeof(MapShmHeader), msg->data.data(), data_size);
        reinterpret_cast<MapShmHeader*>(g_map_mmap)->is_valid = 1;
        reinterpret_cast<MapShmHeader*>(g_map_mmap)->sequence++;
    }

    LOG_FIRST_N(INFO, 1) << "Map received: " << header.width << "x" << header.height;
}

void ControlLoop() {
    ros::Rate rate(100);
    double dt = 0.01;
    double target_vel = 0.0;

    bool control_ready = g_control_reader.Open(SHM_CONTROL_CMD);

    while (g_running && ros::ok()) {
        if (!control_ready) {
            control_ready = g_control_reader.Open(SHM_CONTROL_CMD);
            if (control_ready) {
                LOG(INFO) << "ControlCmd SHM connected!";
            }
        }

        ControlCmdData cmd;
        if (control_ready && g_control_reader.ReadLatest(cmd)) {
            target_vel += cmd.acceleration * dt;
            target_vel = std::max(0.0, std::min(3.0, target_vel));

            double angular_z = target_vel * std::tan(cmd.steering_angle) / kWheelBase;
            angular_z = std::max(-1.0, std::min(1.0, angular_z));

            geometry_msgs::Twist twist;
            twist.linear.x = target_vel;
            twist.angular.z = angular_z;
            g_cmd_vel_pub->publish(twist);

            // 路径可视化
            {
                const PlannerPathData* path_ptr = g_path_reader.Get();
                if (path_ptr && path_ptr->is_valid && path_ptr->sequence != g_last_path_seq && path_ptr->path_size > 0) {
                    g_last_path_seq = path_ptr->sequence;
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

        } else {
            if (target_vel > 0.01) {
                target_vel = std::max(0.0, target_vel - 0.5 * dt);
                geometry_msgs::Twist twist;
                twist.linear.x = target_vel;
                twist.angular.z = 0.0;
                g_cmd_vel_pub->publish(twist);
            }
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

    ros::init(argc, argv, "gazebo_bridge", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    // 共享内存通道
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

    // 地图 SHM
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
        std::memset(g_map_mmap, 0, max_map_size);
    }

    // Gazebo 传送服务
    ros::ServiceClient set_model_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    g_set_model_client = &set_model_client;

    // Subscribers
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, OdomCallback);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, GoalCallback);
    ros::Subscriber init_sub = nh.subscribe("/initialpose", 1, InitPoseCallback);
    ros::Subscriber map_sub = nh.subscribe("/map", 1, MapCallback);

    // Publishers
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    g_cmd_vel_pub = &cmd_vel_pub;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/planned_path", 1);
    g_path_pub = &path_pub;

    g_path_reader.Open(SHM_PLANNER_PATH);

    LOG(INFO) << "Gazebo Bridge ready. Entering control loop...";

    ControlLoop();

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
