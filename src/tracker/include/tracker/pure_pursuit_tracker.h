#ifndef HYBRID_ASTAR_TRACKER_PURE_PURSUIT_TRACKER_H
#define HYBRID_ASTAR_TRACKER_PURE_PURSUIT_TRACKER_H

#include "i_tracker.h"
#include "common/types.h"

#include <vector>
#include <cmath>

/**
 * @brief Pure Pursuit 路径跟踪器 — 最简单的几何跟踪算法
 *
 * 算法原理:
 *   假设车辆在当前速度下沿圆弧运动, 通过几何关系计算使车辆
 *   经过"前视距离"后到达路径上某一点所需的转向角。
 *
 * 算法流程 (每帧 ~100Hz):
 *   1. FindClosestPoint:    在路径上找到距离车辆最近的点索引
 *   2. FindLookaheadPoint:  从最近点向前找前视距离 Ld 处的目标点
 *   3. 变换到车辆坐标系:     计算目标点在车辆局部坐标中的横向偏移 local_y
 *   4. Pure Pursuit 公式:    曲率 κ = 2·local_y / Ld²
 *                          转向角 δ = atan2(κ·轴距, 1)
 *   5. 转向平滑:              低通滤波 + 速率限制 (防突变)
 *   6. 曲率自适应速度:        弯道减速 (desired = target / (1 + k·|κ|))
 *   7. 速度 P 控制:           accel = kp * (desired - current)
 *
 * 优点: 实现简单, 计算量极小 (< 1μs)
 * 缺点: 无预测能力, 弯道性能不如 MPC
 *
 * 这是最简单的跟踪算法, 后续可替换为完整 MPC 控制器。
 */
class PurePursuitTracker : public ITracker {
public:
    PurePursuitTracker() = default;
    ~PurePursuitTracker() override = default;

    // ---- ITracker 接口实现 ----

    bool Init(const TrackerConfig& cfg) override;

    void SetPath(const Path& path) override;

    /**
     * @brief 计算控制指令 (核心算法)
     *
     * 实现完整的 Pure Pursuit + 自适应速度控制
     */
    ControlCommand Compute(const VehicleState& state) override;

    /**
     * @brief 判断是否到达终点 (当前距离终点 < goal_tolerance_xy)
     */
    bool IsGoalReached(const VehicleState& state) override;

    void Reset() override;

    /// 是否已设置有效路径
    bool HasPath() const { return path_set_; }

private:
    /**
     * @brief 在路径上搜索距离车辆最近的点 (欧氏距离)
     * @param x, y 车辆当前位置 (世界坐标)
     * @return 最近路径点的索引
     *
     * 实现: 线性遍历 (O(N)), 路径通常 < 1000 点, 足够快
     * 优化空间: 可以从上一帧的最近点附近搜索 (车辆每帧移动 < 0.05m)
     */
    size_t FindClosestPoint(double x, double y) const;

    /**
     * @brief 从最近点开始沿路径向前搜索, 找到距离 >= lookahead 的第一个点
     * @param state 车辆当前状态 (用于距离测量参考)
     * @param closest_idx 最近点索引
     * @param lookahead 前视距离 (米)
     * @return 前视目标点
     *
     * Bug #21b 修复: 使用 state.x/state.y (车辆当前位置) 作为距离参考,
     * 而非 path_[closest_idx] 的坐标, 这是标准 Pure Pursuit 的做法
     */
    PathPoint FindLookaheadPoint(const VehicleState& state, size_t closest_idx, double lookahead) const;

    TrackerConfig cfg_;                // 配置参数
    Path path_;                        // 当前跟踪路径
    bool path_set_ = false;            // 是否已设置路径
    double last_steering_ = 0.0;       // 上一帧转向角 (用于低通滤波/速率限制)
};

#endif // HYBRID_ASTAR_TRACKER_PURE_PURSUIT_TRACKER_H
