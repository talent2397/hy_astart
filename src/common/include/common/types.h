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

/**
 * @brief 二维位姿 (x, y, 偏航角)
 *
 * 坐标系: 右手系, X 轴正方向为前方, theta 逆时针为正
 * theta = 0 时车辆朝 X 轴正方向; theta = π/2 时朝 Y 轴正方向
 */
struct Pose2D
{
    double x = 0.0;      // 世界坐标系 X 坐标 (米)
    double y = 0.0;      // 世界坐标系 Y 坐标 (米)
    double theta = 0.0;  // 偏航角 (弧度), 范围通常 (-π, π]

    Pose2D() = default;
    Pose2D(double x, double y, double theta) : x(x), y(y), theta(theta) {}

    /// 提取位置为 Eigen 2D 向量
    Eigen::Vector2d Position() const { return Eigen::Vector2d(x, y); }
};

/**
 * @brief 二维速度 (线速度 + 角速度)
 */
struct Twist2D
{
    double linear_x = 0.0;   // 前进方向线速度 (m/s)
    double angular_z = 0.0;  // 绕 Z 轴角速度 (rad/s), 逆时针为正
};

/**
 * @brief 控制指令 — 发送给车辆底层执行器
 *
 * 由 Tracker 计算, 经 SHM 传给 Bridge, Bridge 转换为 cmd_vel 发给 Gazebo
 */
struct ControlCommand
{
    double acceleration = 0.0;    // 加速度 (m/s²), 正值加速/负值减速
    double steering_angle = 0.0;  // 前轮转向角 (弧度), 正值左转/负值右转
    uint64_t timestamp = 0;       // 时间戳 (纳秒), 用于判断数据有效性

    bool IsValid() const
    {
        return timestamp > 0;
    }
};

/**
 * @brief 车辆完整状态 — 位姿 + 速度
 *
 * 这是系统中传递的核心车辆状态数据, 由 Bridge 从 Gazebo 获取真实值并写入 SHM
 */
struct VehicleState
{
    // ---- 位姿 ----
    double x = 0.0;              // 世界 X (米)
    double y = 0.0;              // 世界 Y (米)
    double theta = 0.0;          // 偏航角 (弧度)

    // ---- 速度 ----
    double linear_velocity = 0.0;   // 前进方向线速度 (m/s), 已投影到车辆坐标系
    double angular_velocity = 0.0;  // 角速度 (rad/s)
    double steering_angle = 0.0;    // 当前转向角 (弧度)

    // ---- 辅助 ----
    uint64_t timestamp = 0;      // 时间戳 (纳秒)

    VehicleState() = default;
    VehicleState(double x, double y, double theta, double v, double w)
        : x(x), y(y), theta(theta), linear_velocity(v), angular_velocity(w) {}

    Pose2D GetPose() const { return Pose2D(x, y, theta); }
    bool IsValid() const { return timestamp > 0; }
};

// ============================================================
// 路径类型
// ============================================================

/**
 * @brief 路径上的一个采样点
 *
 * 路径 = 一系列 PathPoint, 由 Planner 生成, 经 SHM 传给 Tracker 跟踪
 */
struct PathPoint
{
    double x = 0.0;      // 世界 X (米)
    double y = 0.0;      // 世界 Y (米)
    double theta = 0.0;  // 期望朝向 (弧度)
    double v_ref = 0.0;  // 该点参考速度 (m/s), 弯道处应偏低
};

/// 路径就是 PathPoint 的有序序列, 从起点到终点
using Path = std::vector<PathPoint>;

// ============================================================
// 地图类型
// ============================================================

/**
 * @brief 占用栅格地图
 *
 * 数据来源: ROS map_server 加载 PGM 地图, 经 Bridge 写入 SHM
 * grid 坐标系: 左下角为原点, X 向右, Y 向上
 */
struct MapData
{
    std::vector<uint8_t> data;  // 占用栅格数据 (0 = 空闲, >0 = 障碍物)
    int width = 0;              // 栅格宽度 (列数)
    int height = 0;             // 栅格高度 (行数)
    double resolution = 0.1;    // 每个栅格的边长 (米/格)
    double origin_x = 0.0;      // 地图原点 (grid(0,0)) 的世界 X 坐标
    double origin_y = 0.0;      // 地图原点 (grid(0,0)) 的世界 Y 坐标

    bool IsValid() const { return width > 0 && height > 0 && !data.empty(); }

    /**
     * @brief 世界坐标 → 栅格索引
     * @return true 表示索引在地图范围内
     */
    bool WorldToGrid(double wx, double wy, int &gx, int &gy) const
    {
        gx = static_cast<int>((wx - origin_x) / resolution);
        gy = static_cast<int>((wy - origin_y) / resolution);
        return (gx >= 0 && gx < width && gy >= 0 && gy < height);
    }

    /**
     * @brief 栅格索引 → 世界坐标 (返回栅格中心点)
     */
    void GridToWorld(int gx, int gy, double &wx, double &wy) const
    {
        wx = (gx + 0.5) * resolution + origin_x;
        wy = (gy + 0.5) * resolution + origin_y;
    }

    /**
     * @brief 检查指定栅格是否有障碍物
     * @note 越界视为障碍物 (保守策略, 防止规划出地图)
     */
    bool IsOccupied(int gx, int gy) const
    {
        if (gx < 0 || gx >= width || gy < 0 || gy >= height)
            return true;  // 地图外 = 障碍物
        return data[gy * width + gx] > 0;
    }
};

// ============================================================
// 配置类型
// ============================================================

/**
 * @brief 规划器配置参数 (Hybrid A*)
 *
 * 这些参数直接影响搜索行为和路径质量
 * TODO: 后续改为从 YAML 文件加载
 */
struct PlannerConfig
{
    double steering_angle = 15.0;             // 最大转向角 (度), 决定最小转弯半径
    int steering_angle_discrete_num = 1;       // 转向角离散份数 (每份 = max/N)
    double wheel_base = 2.0;                  // 轴距 (米), 自行车模型参数
    double segment_length = 1.6;              // 搜索步长 (米), 每一步模拟的距离
    int segment_length_discrete_num = 8;       // 每步内的细分数 (碰撞检测粒度)
    double steering_penalty = 1.5;            // 转向惩罚系数 (鼓励走直线)
    double reversing_penalty = 3.0;           // 倒车惩罚系数 (尽量避免倒车)
    double steering_change_penalty = 2.0;     // 转向变化惩罚 (鼓励平滑转向)
    double shot_distance = 5.0;               // RS 捷径触发距离 (米), 终点在此范围内尝试 RS 直连
    int grid_size_phi = 72;                   // 角度网格数 (72 → 5° 分辨率)
    double state_grid_resolution = 1.0;       // 状态空间网格分辨率 (米)
    int thread_pool_size = 4;                 // 线程池大小 (0 = 自动)
};

/**
 * @brief 跟踪器配置参数 (Pure Pursuit / 后续 MPC)
 *
 * 控制算法的参数, 影响跟踪精度和车辆行为
 */
struct TrackerConfig
{
    double lookahead_distance = 3.0;    // 前视距离基准 (米), 实际值 = base + 速度系数*|v|
    double target_velocity = 5.0;       // 目标速度 (m/s), 直线段满速
    double max_steering = 0.6;          // 最大转向角 (弧度 ≈ 34°)
    double wheel_base = 2.0;            // 轴距 (米), 与 Planner 保持一致
    double goal_tolerance_xy = 0.2;     // 终点判定距离 (米), 在此范围内认为到达
    double goal_tolerance_theta = 0.1;  // 终点判定角度 (弧度)
    double kp_velocity = 0.5;           // 速度 P 控制器比例增益
};

// ============================================================
// 类型别名 (兼容原项目 Eigen 类型, 简化代码书写)
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

/// 使用对齐分配器的 Eigen 向量容器 (Eigen 要求固定大小类型必须对齐)
template <int dim>
using TypeVectorVecd = typename std::vector<Eigen::Matrix<double, dim, 1>,
                                            Eigen::aligned_allocator<Eigen::Matrix<double, dim, 1>>>;

using VectorVec4d = TypeVectorVecd<4>;
using VectorVec3d = TypeVectorVecd<3>;
using VectorVec2d = TypeVectorVecd<2>;

#endif // HYBRID_ASTAR_COMMON_TYPES_H
