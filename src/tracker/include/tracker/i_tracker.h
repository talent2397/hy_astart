#ifndef HYBRID_ASTAR_TRACKER_ITRACKER_H
#define HYBRID_ASTAR_TRACKER_ITRACKER_H

#include "common/types.h"

/**
 * @brief 轨迹跟踪器抽象接口
 *
 * 所有跟踪算法（Pure Pursuit, MPC, Stanley, LQR 等）都实现此接口。
 * 后续替换跟踪算法只需实现这个接口即可。
 */
class ITracker {
public:
    virtual ~ITracker() = default;

    /**
     * @brief 初始化跟踪器
     * @param cfg 跟踪器配置参数
     * @return true 成功
     */
    virtual bool Init(const TrackerConfig& cfg) = 0;

    /**
     * @brief 设置跟踪路径
     * @param path 规划器输出的路径
     */
    virtual void SetPath(const Path& path) = 0;

    /**
     * @brief 计算控制指令
     * @param state 车辆当前状态 (x, y, theta, v, ...)
     * @return 控制指令 (acceleration, steering_angle)
     */
    virtual ControlCommand Compute(const VehicleState& state) = 0;

    /**
     * @brief 判断是否到达终点
     * @param state 车辆当前状态
     * @return true 已到达
     */
    virtual bool IsGoalReached(const VehicleState& state) = 0;

    /**
     * @brief 重置跟踪器
     */
    virtual void Reset() = 0;
};

#endif // HYBRID_ASTAR_TRACKER_ITRACKER_H
