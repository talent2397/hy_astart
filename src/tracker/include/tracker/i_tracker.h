#ifndef HYBRID_ASTAR_TRACKER_ITRACKER_H
#define HYBRID_ASTAR_TRACKER_ITRACKER_H

#include "common/types.h"

/**
 * @brief 轨迹跟踪器抽象接口 (策略模式)
 *
 * 所有跟踪算法 (Pure Pursuit, MPC, Stanley, LQR 等) 都实现此接口。
 * 后续替换跟踪算法只需:
 *   1. 实现 ITracker 接口
 *   2. 修改 run_tracker.cpp 中的实例化语句
 *   3. 其余代码 (IPC, Bridge, Planner) 无需任何修改
 *
 * 核心方法: Init() → SetPath() → Compute() (循环调用 @100Hz) → IsGoalReached()
 *
 * 设计说明:
 *   - SetPath() 和 Compute() 解耦: 路径可以低频更新 (~1Hz), 控制高频计算 (100Hz)
 *   - Compute() 是无状态的纯计算: 输入 VehicleState, 输出 ControlCommand
 */
class ITracker {
public:
    virtual ~ITracker() = default;

    /**
     * @brief 初始化跟踪器 (设置参数)
     * @param cfg 跟踪器配置 (前视距离, 目标速度, 最大转向角等)
     * @return true 初始化成功
     */
    virtual bool Init(const TrackerConfig& cfg) = 0;

    /**
     * @brief 设置/更新跟踪路径
     * @param path 规划器输出的新路径 (PathPoint 序列)
     *
     * 调用时机:
     *   - Tracker 启动后首次收到 Planner 的 PATH_READY 消息
     *   - Planner 重规划完成后再次收到 PATH_READY 消息
     */
    virtual void SetPath(const Path& path) = 0;

    /**
     * @brief 计算当前时刻的控制指令 (核心方法, 高频调用)
     * @param state 车辆当前状态 (来自 SHM vehicle_state)
     * @return 控制指令 (加速度 + 转向角)
     *
     * 调用频率: 100Hz (主控制循环)
     * 注意: 此函数应保持轻量, 避免在循环内做大量分配
     */
    virtual ControlCommand Compute(const VehicleState& state) = 0;

    /**
     * @brief 判断是否到达终点
     * @param state 车辆当前状态
     * @return true 已到达 (距离终点 < goal_tolerance_xy)
     *
     * 调用时机: 每次 Compute() 之后
     * 到达后 Tracker 应减速到零并等待新路径
     */
    virtual bool IsGoalReached(const VehicleState& state) = 0;

    /**
     * @brief 重置跟踪器 (清理路径和内部状态)
     */
    virtual void Reset() = 0;
};

#endif // HYBRID_ASTAR_TRACKER_ITRACKER_H
