#ifndef HYBRID_ASTAR_PLANNER_HYBRID_A_STAR_H
#define HYBRID_ASTAR_PLANNER_HYBRID_A_STAR_H

#include "rs_path.h"
#include "state_node.h"
#include "common/types.h"

#include <glog/logging.h>
#include <map>
#include <memory>

/**
 * @brief Hybrid A* 核心搜索算法 (去 ROS 依赖)
 *
 * 算法概述:
 *   Hybrid A* = 标准 A* + 运动学约束
 *
 *   标准 A* 在 (x, y) 网格中搜索, 邻居是四/八连通。
 *   Hybrid A* 在 (x, y, θ) 状态空间中搜索, 邻居通过自行车运动模型生成:
 *     - 对每个可能的转向角 φ ∈ {-max, ..., 0, ..., +max}
 *     - 模拟车辆前进/后退 segment_length 米
 *     - 生成新的 (x, y, θ) 状态
 *
 *   关键优化:
 *   1. RS 曲线捷径 (shot_distance): 当节点距离终点 < shot_distance 时,
 *      尝试用 Reeds-Shepp 曲线直连, 大幅加速收敛
 *   2. 状态空间离散化: 用 3D 网格 (x, y, φ) 去重, 相同网格只保留代价最小的节点
 *   3. 碰撞检测: 用矩形车辆模型, 沿运动路径细分检测
 *
 * 参考: "Practical Search Techniques in Path Planning for Autonomous Driving"
 *       (Dolgov et al., 2008)
 */
class HybridAStar {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    HybridAStar() = delete;

    /**
     * @brief 构造 Hybrid A* 搜索器
     * @param steering_angle               最大转向角 (度)
     * @param steering_angle_discrete_num  转向角离散份数
     * @param segment_length               搜索步长 (米)
     * @param segment_length_discrete_num   步长内细分段数
     * @param wheel_base                   轴距 (米)
     * @param steering_penalty             转向惩罚系数
     * @param reversing_penalty            倒车惩罚系数
     * @param steering_change_penalty      转向变化惩罚系数
     * @param shot_distance                RS 捷径触发距离 (米)
     * @param grid_size_phi                角度网格数 (72 → 5° 分辨率)
     */
    HybridAStar(double steering_angle, int steering_angle_discrete_num, double segment_length,
                int segment_length_discrete_num, double wheel_base, double steering_penalty,
                double reversing_penalty, double steering_change_penalty, double shot_distance,
                int grid_size_phi = 72);

    ~HybridAStar();

    /**
     * @brief 初始化搜索空间 (设置地图边界和网格分辨率)
     * @param x_lower, x_upper  世界 X 坐标范围
     * @param y_lower, y_upper  世界 Y 坐标范围
     * @param state_grid_resolution  状态空间网格分辨率 (米)
     * @param map_grid_resolution    地图网格分辨率 (米/格)
     */
    void Init(double x_lower, double x_upper, double y_lower, double y_upper,
              double state_grid_resolution, double map_grid_resolution = 0.1);

    /**
     * @brief 执行 A* 搜索
     * @param start_state  起点 (x, y, θ)
     * @param goal_state   终点 (x, y, θ)
     * @return true 找到路径 (结果可通过 GetPath() 获取)
     */
    bool Search(const Vec3d& start_state, const Vec3d& goal_state);

    /// 获取搜索树 (所有展开过的边, 用于可视化)
    VectorVec4d GetSearchedTree();

    /// 获取规划路径 (反向回溯 parent_node_ 链表)
    VectorVec3d GetPath() const;

    int GetVisitedNodesNumber() const { return visited_node_number_; }

    double GetPathLength() const;

    // ---- 坐标转换工具 ----
    Vec2d CoordinateRounding(const Vec2d& pt) const;
    Vec2i Coordinate2MapGridIndex(const Vec2d& pt) const;

    // ---- 障碍物设置 ----
    void SetObstacle(double pt_x, double pt_y);       // 世界坐标 → 栅格
    void SetObstacle(unsigned int x, unsigned int y);  // 栅格索引

    /// 设置车辆矩形尺寸 (用于碰撞检测)
    /// @param length        车长 (米)
    /// @param width         车宽 (米)
    /// @param rear_axle_dist 后轴中心到车尾距离 (米)
    void SetVehicleShape(double length, double width, double rear_axle_dist);

    /// 重置搜索状态 (清理 openset/closeset, 保留地图)
    void Reset();

private:
    // ---- 碰撞检测 (Bresenham 线段 + 矩形车辆模型) ----
    inline bool HasObstacle(int grid_index_x, int grid_index_y) const;
    inline bool HasObstacle(const Vec2i& grid_index) const;
    bool CheckCollision(const double& x, const double& y, const double& theta);
    inline bool LineCheck(double x0, double y0, double x1, double y1);

    // ---- RS 捷径: 尝试用 Reeds-Shepp 曲线直接连到终点 ----
    bool AnalyticExpansions(const StateNode::Ptr& current_node_ptr,
                            const StateNode::Ptr& goal_node_ptr, double& length);

    // ---- A* 代价函数 ----
    /// g(n): 从起点到 neighbor 的实际代价 (考虑转向/倒车惩罚)
    inline double ComputeG(const StateNode::Ptr& current_node_ptr,
                           const StateNode::Ptr& neighbor_node_ptr) const;
    /// h(n): 从当前节点到终点的启发式估计 (L1 距离 或 RS 距离)
    inline double ComputeH(const StateNode::Ptr& current_node_ptr,
                           const StateNode::Ptr& terminal_node_ptr);

    // ---- 空间离散化 ----
    /// 连续状态 → 3D 网格索引 (x, y, θ)
    inline Vec3i State2Index(const Vec3d& state) const;
    /// 地图网格索引 → 世界坐标 (栅格中心)
    inline Vec2d MapGridIndex2Coordinate(const Vec2i& grid_index) const;

    // ---- 邻居生成 ----
    /// 从当前节点出发, 生成所有可能的邻居节点 (前进 + 后退, 各转向角)
    void GetNeighborNodes(const StateNode::Ptr& curr_node_ptr,
                          std::vector<StateNode::Ptr>& neighbor_nodes);

    // ---- 自行车运动模型 ----
    /// 用运动学模型向前模拟一步: x += v*cos(θ)*dt, y += v*sin(θ)*dt, θ += v/L*tan(φ)*dt
    inline void DynamicModel(const double& step_size, const double& phi,
                             double& x, double& y, double& theta) const;

    // ---- 工具函数 ----
    static inline double Mod2Pi(const double& x);   // 角度归一化到 (-π, π]
    bool BeyondBoundary(const Vec2d& pt) const;      // 检查是否超出地图
    void ReleaseMemory();                            // 释放所有动态内存

private:
    // ---- 地图数据 ----
    uint8_t* map_data_ = nullptr;       // 障碍物栅格 (0=空闲, 1=障碍物)

    // ---- 网格参数 ----
    double STATE_GRID_RESOLUTION_{}, MAP_GRID_RESOLUTION_{};
    double ANGULAR_RESOLUTION_{};        // 每个角度网格对应的弧度 (360°/N * π/180)
    int STATE_GRID_SIZE_X_{}, STATE_GRID_SIZE_Y_{}, STATE_GRID_SIZE_PHI_{};
    int MAP_GRID_SIZE_X_{}, MAP_GRID_SIZE_Y_{};

    double map_x_lower_{}, map_x_upper_{}, map_y_lower_{}, map_y_upper_{};

    // ---- 搜索数据结构 ----
    StateNode::Ptr terminal_node_ptr_ = nullptr;        // 终点节点 (路径回溯起点)
    StateNode::Ptr*** state_node_map_ = nullptr;       // 3D 离散网格: [x][y][phi] → StateNode*

    /// openset: key=f_cost, value=节点指针
    /// multimap 允许相同 f_cost 的多个节点, 按 key 排序, begin() 是最小 f_cost
    std::multimap<double, StateNode::Ptr> openset_;

    // ---- 车辆运动学参数 ----
    double wheel_base_;                  // 轴距 L (米)
    double segment_length_;              // 搜索步长 (米)
    double move_step_size_;              // 每小步距离 = segment_length / segment_length_discrete_num
    double steering_radian_step_size_;   // 转向角增量 = max_steering / num
    double steering_radian_;             // 最大转向角 (弧度)
    double tie_breaker_;                 // 打破平局的微小系数 (1.0 + 1e-3)

    // ---- 惩罚系数 ----
    double shot_distance_;               // RS 捷径触发距离
    int segment_length_discrete_num_;    // 步长细分数
    int steering_discrete_num_;          // 转向角离散数
    double steering_penalty_;            // 转向惩罚
    double reversing_penalty_;           // 倒车惩罚
    double steering_change_penalty_;     // 转向变化惩罚

    double path_length_ = 0.0;           // 规划路径总长

    std::shared_ptr<RSPath> rs_path_ptr_;  // RS 曲线计算器

    // ---- 车辆碰撞模型 ----
    VecXd vehicle_shape_;               // 4 个顶点 (8 个坐标值)
    MatXd vehicle_shape_discrete_;      // 离散化的车辆轮廓点 (用于碰撞检测)

    // ---- 调试统计 ----
    double check_collision_use_time = 0.0;
    int num_check_collision = 0;
    int visited_node_number_ = 0;        // 已访问节点数
};

#endif // HYBRID_ASTAR_PLANNER_HYBRID_A_STAR_H
