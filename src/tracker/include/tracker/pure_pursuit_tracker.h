#ifndef HYBRID_ASTAR_TRACKER_PURE_PURSUIT_TRACKER_H
#define HYBRID_ASTAR_TRACKER_PURE_PURSUIT_TRACKER_H

#include "i_tracker.h"
#include "common/types.h"

#include <vector>
#include <cmath>

/**
 * @brief Pure Pursuit 路径跟踪器（最简单的实现）
 *
 * 算法流程：
 *   1. 在路径上找到距离车辆前视距离 Ld 处的目标点
 *   2. 根据自行车模型计算转向角：
 *        δ = atan2(2 * L * sin(α) / Ld)
 *      其中 L = 轴距, Ld = 前视距离, α = 车辆朝向与目标点方向的夹角
 *   3. 速度：P 控制器跟踪目标速度
 *
 * 这是最简单的跟踪算法，后续可替换为完整 MPC。
 */
class PurePursuitTracker : public ITracker {
public:
    PurePursuitTracker() = default;
    ~PurePursuitTracker() override = default;

    bool Init(const TrackerConfig& cfg) override;
    void SetPath(const Path& path) override;
    ControlCommand Compute(const VehicleState& state) override;
    bool IsGoalReached(const VehicleState& state) override;
    void Reset() override;

private:
    /**
     * @brief 在路径上找到距离车辆最近的点索引
     * @param x, y 车辆位置
     * @return 最近路径点的索引
     */
    size_t FindClosestPoint(double x, double y) const;

    /**
     * @brief 从最近点开始，沿路径向前找前视距离处的目标点
     * @param closest_idx 最近点索引
     * @param lookahead 前视距离
     * @return 目标路径点
     */
    PathPoint FindLookaheadPoint(size_t closest_idx, double lookahead) const;

    TrackerConfig cfg_;
    Path path_;
    bool path_set_ = false;
};

#endif // HYBRID_ASTAR_TRACKER_PURE_PURSUIT_TRACKER_H
