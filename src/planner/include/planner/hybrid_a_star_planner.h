#ifndef HYBRID_ASTAR_PLANNER_HYBRID_A_STAR_PLANNER_H
#define HYBRID_ASTAR_PLANNER_HYBRID_A_STAR_PLANNER_H

#include "i_planner.h"
#include "hybrid_a_star.h"
#include "common/thread_pool.h"

#include <memory>

/**
 * @brief Hybrid A* 规划器 — IPlanner 接口的实现类 (适配器模式)
 *
 * 职责: 将 HybridAStar 核心搜索算法包装为 IPlanner 接口, 处理:
 *   1. 配置参数转换为 HybridAStar 构造参数
 *   2. MapData (通用类型) → 内部障碍物数据的转换
 *   3. VectorVec3d (Eigen 路径) → Path (通用路径类型) 的转换
 *
 * 后续如需换规划算法 (如 RRT*, Lattice), 只需:
 *   写一个新的 IPlanner 实现类, 在 run_planner.cpp 切换实例化即可
 */
class HybridAStarPlanner : public IPlanner {
public:
    HybridAStarPlanner() = default;
    ~HybridAStarPlanner() override = default;

    // ---- IPlanner 接口实现 ----

    bool Init(const PlannerConfig& cfg) override;

    bool Plan(const Pose2D& start, const Pose2D& goal,
              const MapData& map, Path& out_path) override;

    VectorVec4d GetSearchTree() const override;

    int GetVisitedNodes() const override;

    void Reset() override;

private:
    /// 将通用 MapData 加载到 HybridAStar 内部 (设置边界/障碍物/车辆形状)
    void LoadMap(const MapData& map);

    PlannerConfig config_;                           // 保存的配置
    std::unique_ptr<HybridAStar> searcher_;           // 核心搜索算法实例
    double map_x_lower_ = 0, map_x_upper_ = 0;       // 地图世界坐标边界 X
    double map_y_lower_ = 0, map_y_upper_ = 0;       // 地图世界坐标边界 Y
};

#endif // HYBRID_ASTAR_PLANNER_HYBRID_A_STAR_PLANNER_H
