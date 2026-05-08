#ifndef HYBRID_ASTAR_PLANNER_HYBRID_A_STAR_PLANNER_H
#define HYBRID_ASTAR_PLANNER_HYBRID_A_STAR_PLANNER_H

#include "i_planner.h"
#include "hybrid_a_star.h"
#include "common/thread_pool.h"

#include <memory>

/**
 * @brief Hybrid A* 规划器 — IPlanner 接口实现
 *
 * 将 HybridAStar 核心算法包装为 IPlanner 接口。
 * 后续如需换规划算法（如 RRT*, Lattice），只需实现 IPlanner 即可。
 */
class HybridAStarPlanner : public IPlanner {
public:
    HybridAStarPlanner() = default;
    ~HybridAStarPlanner() override = default;

    // ---- IPlanner 接口 ----
    bool Init(const PlannerConfig& cfg) override;
    bool Plan(const Pose2D& start, const Pose2D& goal,
              const MapData& map, Path& out_path) override;
    VectorVec4d GetSearchTree() const override;
    int GetVisitedNodes() const override;
    void Reset() override;

private:
    // 将 MapData 加载到 HybridAStar 内部
    void LoadMap(const MapData& map);

    PlannerConfig config_;
    std::unique_ptr<HybridAStar> searcher_;
    double map_x_lower_ = 0, map_x_upper_ = 0;
    double map_y_lower_ = 0, map_y_upper_ = 0;
};

#endif // HYBRID_ASTAR_PLANNER_HYBRID_A_STAR_PLANNER_H
