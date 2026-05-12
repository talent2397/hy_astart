#ifndef HYBRID_ASTAR_PLANNER_IPLANNER_H
#define HYBRID_ASTAR_PLANNER_IPLANNER_H

#include "common/types.h"

/**
 * @brief 规划器抽象接口 (策略模式)
 *
 * 所有路径规划算法 (Hybrid A*, RRT, RRT*, Lattice, A* 等) 都实现此接口。
 * 后续替换规划算法只需:
 *   1. 实现 IPlanner 接口
 *   2. 修改 run_planner.cpp 中的实例化语句
 *   3. 其余代码 (IPC, Bridge, Tracker) 无需任何修改
 *
 * 核心方法: Init() → Plan() (可多次调用) → Reset() (清理这次搜索的内部状态)
 */
class IPlanner {
public:
    virtual ~IPlanner() = default;

    /**
     * @brief 初始化规划器 (设置参数, 分配资源)
     * @param cfg 规划器配置 (转向角, 步长, 惩罚系数等)
     * @return true 初始化成功
     */
    virtual bool Init(const PlannerConfig& cfg) = 0;

    /**
     * @brief 执行一次路径规划
     * @param start   起点位姿 (世界坐标)
     * @param goal    终点位姿 (世界坐标)
     * @param map     障碍物地图 (占用栅格)
     * @param out_path 输出路径 (PathPoint 序列从起点到终点)
     * @return true 找到可行路径
     */
    virtual bool Plan(const Pose2D& start, const Pose2D& goal,
                      const MapData& map, Path& out_path) = 0;

    /**
     * @brief 获取搜索过程中展开的搜索树 (用于 RViz 可视化调试)
     * @return 搜索树边集合, 每条边 = [x0, y0, x1, y1]
     */
    virtual VectorVec4d GetSearchTree() const = 0;

    /**
     * @brief 获取本次搜索访问的节点数 (调试/性能分析用)
     */
    virtual int GetVisitedNodes() const = 0;

    /**
     * @brief 重置规划器内部状态 (清理本次搜索的 openset/closeset/搜索树)
     *
     * Plan() 前后均可调用, 确保每次搜索从干净状态开始
     */
    virtual void Reset() = 0;
};

#endif // HYBRID_ASTAR_PLANNER_IPLANNER_H
