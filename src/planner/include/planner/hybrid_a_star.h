#ifndef HYBRID_ASTAR_PLANNER_HYBRID_A_STAR_H
#define HYBRID_ASTAR_PLANNER_HYBRID_A_STAR_H

#include "rs_path.h"
#include "state_node.h"
#include "common/types.h"

#include <glog/logging.h>
#include <map>
#include <memory>

/**
 * @brief Hybrid A* 核心搜索算法
 *
 * 参考原项目，去除了所有 ROS 依赖。
 * 使用自行车模型 + Reeds-Shepp 曲线进行路径搜索。
 */
class HybridAStar {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    HybridAStar() = delete;

    HybridAStar(double steering_angle, int steering_angle_discrete_num, double segment_length,
                int segment_length_discrete_num, double wheel_base, double steering_penalty,
                double reversing_penalty, double steering_change_penalty, double shot_distance,
                int grid_size_phi = 72);

    ~HybridAStar();

    void Init(double x_lower, double x_upper, double y_lower, double y_upper,
              double state_grid_resolution, double map_grid_resolution = 0.1);

    bool Search(const Vec3d& start_state, const Vec3d& goal_state);

    VectorVec4d GetSearchedTree();
    VectorVec3d GetPath() const;
    int GetVisitedNodesNumber() const { return visited_node_number_; }
    double GetPathLength() const;

    Vec2d CoordinateRounding(const Vec2d& pt) const;
    Vec2i Coordinate2MapGridIndex(const Vec2d& pt) const;

    void SetObstacle(double pt_x, double pt_y);
    void SetObstacle(unsigned int x, unsigned int y);

    void SetVehicleShape(double length, double width, double rear_axle_dist);
    void Reset();

private:
    inline bool HasObstacle(int grid_index_x, int grid_index_y) const;
    inline bool HasObstacle(const Vec2i& grid_index) const;
    bool CheckCollision(const double& x, const double& y, const double& theta);
    inline bool LineCheck(double x0, double y0, double x1, double y1);

    bool AnalyticExpansions(const StateNode::Ptr& current_node_ptr,
                            const StateNode::Ptr& goal_node_ptr, double& length);

    inline double ComputeG(const StateNode::Ptr& current_node_ptr,
                           const StateNode::Ptr& neighbor_node_ptr) const;
    inline double ComputeH(const StateNode::Ptr& current_node_ptr,
                           const StateNode::Ptr& terminal_node_ptr);

    inline Vec3i State2Index(const Vec3d& state) const;
    inline Vec2d MapGridIndex2Coordinate(const Vec2i& grid_index) const;

    void GetNeighborNodes(const StateNode::Ptr& curr_node_ptr,
                          std::vector<StateNode::Ptr>& neighbor_nodes);

    inline void DynamicModel(const double& step_size, const double& phi,
                             double& x, double& y, double& theta) const;

    static inline double Mod2Pi(const double& x);
    bool BeyondBoundary(const Vec2d& pt) const;
    void ReleaseMemory();

private:
    uint8_t* map_data_ = nullptr;
    double STATE_GRID_RESOLUTION_{}, MAP_GRID_RESOLUTION_{};
    double ANGULAR_RESOLUTION_{};
    int STATE_GRID_SIZE_X_{}, STATE_GRID_SIZE_Y_{}, STATE_GRID_SIZE_PHI_{};
    int MAP_GRID_SIZE_X_{}, MAP_GRID_SIZE_Y_{};

    double map_x_lower_{}, map_x_upper_{}, map_y_lower_{}, map_y_upper_{};

    StateNode::Ptr terminal_node_ptr_ = nullptr;
    StateNode::Ptr*** state_node_map_ = nullptr;

    std::multimap<double, StateNode::Ptr> openset_;

    double wheel_base_;
    double segment_length_;
    double move_step_size_;
    double steering_radian_step_size_;
    double steering_radian_;
    double tie_breaker_;

    double shot_distance_;
    int segment_length_discrete_num_;
    int steering_discrete_num_;
    double steering_penalty_;
    double reversing_penalty_;
    double steering_change_penalty_;

    double path_length_ = 0.0;

    std::shared_ptr<RSPath> rs_path_ptr_;

    VecXd vehicle_shape_;
    MatXd vehicle_shape_discrete_;

    // debug
    double check_collision_use_time = 0.0;
    int num_check_collision = 0;
    int visited_node_number_ = 0;
};

#endif // HYBRID_ASTAR_PLANNER_HYBRID_A_STAR_H
