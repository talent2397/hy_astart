#ifndef HYBRID_ASTAR_COMMON_TYPES_H
#define HYBRID_ASTAR_COMMON_TYPES_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <cstdint>
#include <string>

// ============================================================
// 基础几何类型
// ============================================================

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;  // yaw angle, radians

    Pose2D() = default;
    Pose2D(double x, double y, double theta) : x(x), y(y), theta(theta) {}

    Eigen::Vector2d Position() const { return Eigen::Vector2d(x, y); }
};

struct Twist2D {
    double linear_x = 0.0;   // m/s
    double angular_z = 0.0;  // rad/s
};

struct ControlCommand {
    double acceleration = 0.0;     // m/s²
    double steering_angle = 0.0;   // rad, front wheel steering angle
    uint64_t timestamp = 0;        // nanoseconds

    bool IsValid() const {
        return timestamp > 0;
    }
};

struct VehicleState {
    // pose
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    // velocity
    double linear_velocity = 0.0;   // m/s
    double angular_velocity = 0.0;  // rad/s
    double steering_angle = 0.0;    // rad
    // aux
    uint64_t timestamp = 0;

    VehicleState() = default;
    VehicleState(double x, double y, double theta, double v, double w)
        : x(x), y(y), theta(theta), linear_velocity(v), angular_velocity(w) {}

    Pose2D GetPose() const { return Pose2D(x, y, theta); }
    bool IsValid() const { return timestamp > 0; }
};

// ============================================================
// 路径类型
// ============================================================

struct PathPoint {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double v_ref = 0.0;  // reference velocity at this point
};

// Path 就是 PathPoint 的序列
using Path = std::vector<PathPoint>;

// ============================================================
// 地图类型
// ============================================================

struct MapData {
    std::vector<uint8_t> data;       // occupancy grid (0=free, 1=occupied)
    int width = 0;                   // grid cells
    int height = 0;
    double resolution = 0.1;         // meters per cell
    double origin_x = 0.0;           // world coordinate of grid(0,0)
    double origin_y = 0.0;

    bool IsValid() const { return width > 0 && height > 0 && !data.empty(); }

    // 世界坐标 → 网格索引
    bool WorldToGrid(double wx, double wy, int& gx, int& gy) const {
        gx = static_cast<int>((wx - origin_x) / resolution);
        gy = static_cast<int>((wy - origin_y) / resolution);
        return (gx >= 0 && gx < width && gy >= 0 && gy < height);
    }

    // 网格索引 → 世界坐标（网格中心）
    void GridToWorld(int gx, int gy, double& wx, double& wy) const {
        wx = (gx + 0.5) * resolution + origin_x;
        wy = (gy + 0.5) * resolution + origin_y;
    }

    // 检查网格是否有障碍物
    bool IsOccupied(int gx, int gy) const {
        if (gx < 0 || gx >= width || gy < 0 || gy >= height) return true;  // out of bounds = obstacle
        return data[gy * width + gx] > 0;
    }
};

// ============================================================
// 配置类型
// ============================================================

struct PlannerConfig {
    double steering_angle = 15.0;             // degrees
    int steering_angle_discrete_num = 1;
    double wheel_base = 2.0;                  // meters
    double segment_length = 1.6;              // meters
    int segment_length_discrete_num = 8;
    double steering_penalty = 1.5;
    double reversing_penalty = 3.0;
    double steering_change_penalty = 2.0;
    double shot_distance = 5.0;               // RS path trigger distance
    int grid_size_phi = 72;                   // angular grid size
    double state_grid_resolution = 1.0;       // meters
    int thread_pool_size = 4;
};

struct TrackerConfig {
    double lookahead_distance = 3.0;          // Pure Pursuit 前视距离 (m)
    double target_velocity = 1.0;             // 目标速度 (m/s)
    double max_steering = 0.6;                // 最大转向角 (rad)
    double wheel_base = 2.0;                  // 轴距 (m)
    double goal_tolerance_xy = 0.2;           // 到达判定 (m)
    double goal_tolerance_theta = 0.1;        // 到达判定 (rad)
    double kp_velocity = 0.5;                 // 速度 P 控制增益
};

// ============================================================
// 类型别名 (兼容原项目 Eigen 类型)
// ============================================================

using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Vec4d = Eigen::Vector4d;
using Vec2i = Eigen::Vector2i;
using Vec3i = Eigen::Vector3i;
using Mat2d = Eigen::Matrix2d;
using Mat3d = Eigen::Matrix3d;
using MatXd = Eigen::MatrixXd;
using VecXd = Eigen::VectorXd;

template<int dim>
using TypeVectorVecd = typename std::vector<Eigen::Matrix<double, dim, 1>,
        Eigen::aligned_allocator<Eigen::Matrix<double, dim, 1>>>;

using VectorVec4d = TypeVectorVecd<4>;
using VectorVec3d = TypeVectorVecd<3>;
using VectorVec2d = TypeVectorVecd<2>;

#endif // HYBRID_ASTAR_COMMON_TYPES_H
