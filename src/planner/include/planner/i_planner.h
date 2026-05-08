#ifndef HYBRID_ASTAR_PLANNER_IPLANNER_H
#define HYBRID_ASTAR_PLANNER_IPLANNER_H

#include "common/types.h"

/**
 * @brief 规划器抽象接口
 *
 * 所有路径规划算法（Hybrid A*, RRT, Lattice 等）都实现此接口。
 * 后续替换规划算法只需实现这个接口即可，其余代码无需修改。
 */
class IPlanner {
public:
    virtual ~IPlanner() = default;

    /**
     * @brief 初始化规划器
     * @param cfg 规划器配置参数
     * @return true 成功
     */
    virtual bool Init(const PlannerConfig& cfg) = 0;

    /**
     * @brief 执行路径规划
     * @param start 起点 (x, y, theta)
     * @param goal  终点 (x, y, theta)
     * @param map   障碍物地图
     * @param out_path 输出路径
     * @return true 成功找到路径
     */
    virtual bool Plan(const Pose2D& start, const Pose2D& goal,
                      const MapData& map, Path& out_path) = 0;

    /**
     * @brief 获取搜索树（用于可视化调试）
     * @return 搜索树边集合 [x0,y0,x1,y1] * N
     */
    virtual VectorVec4d GetSearchTree() const = 0;

    /**
     * @brief 获取已访问节点数（调试用）
     */
    virtual int GetVisitedNodes() const = 0;

    /**
     * @brief 重置规划器状态
     */
    virtual void Reset() = 0;
};

#endif // HYBRID_ASTAR_PLANNER_IPLANNER_H
