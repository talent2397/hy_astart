/**
 * @file pure_pursuit_tracker.cpp
 * @brief Pure Pursuit 跟踪算法实现
 *
 * 核心公式推导 (自行车模型):
 *
 *   给定前视距离 Ld 处的目标点, 在车辆局部坐标系中有横向偏移 local_y。
 *   根据几何关系, 车辆沿半径为 R 的圆弧运动时:
 *
 *     Ld² = local_x² + local_y²     (前视距离的平方 = 纵向² + 横向²)
 *     又  R² = (R - local_y)² + local_x²
 *     →   2R·local_y = Ld²
 *     →   曲率 κ = 1/R = 2·local_y / Ld²
 *     →   转向角 δ = atan2(κ·L, 1)   (L = 轴距)
 *
 * 自适应前视距离:
 *   Ld = Ld_base + k_v * |v|
 *   速度越高前视越远 (高速看远处, 低速看近处)
 *   范围: [1.0, 4.0] 米
 *
 * 曲率自适应速度:
 *   v_desired = v_target / (1 + k * |κ|)
 *   弯道越多 (|κ| 越大) 速度越慢
 *   系数 k = 10.0, 弯道半径 < 2m 时速度降至 ~30%
 *
 * 转向平滑:
 *   无低通滤波, 改用速率限制 (3.0 rad/s),
 *   保证每帧转向角变化 ≤ 3.0 * 0.01 = 0.03 rad
 */

#include "tracker/pure_pursuit_tracker.h"
#include <glog/logging.h>
#include <algorithm>
#include <limits>

bool PurePursuitTracker::Init(const TrackerConfig& cfg) {
    cfg_ = cfg;
    LOG(INFO) << "PurePursuit ready. lookahead=" << cfg_.lookahead_distance
              << " target_vel=" << cfg_.target_velocity
              << " max_steer=" << cfg_.max_steering;
    return true;
}

void PurePursuitTracker::SetPath(const Path& path) {
    path_ = path;
    path_set_ = !path_.empty();
    if (path_set_) {
        LOG(INFO) << "PurePursuit: received path with " << path_.size() << " points";
    }
}

void PurePursuitTracker::Reset() {
    path_.clear();
    path_set_ = false;
}

/**
 * @brief 在路径上找距离车辆最近的点 (线性搜索, O(N))
 *
 * 使用平方距离比较避免开根号 (性能优化)
 */
size_t PurePursuitTracker::FindClosestPoint(double x, double y) const {
    if (path_.empty()) return 0;

    size_t closest = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < path_.size(); ++i) {
        double dx = path_[i].x - x;
        double dy = path_[i].y - y;
        double dist = dx * dx + dy * dy;  // 平方距离, 避免 sqrt
        if (dist < min_dist) {
            min_dist = dist;
            closest = i;
        }
    }
    return closest;
}

/**
 * @brief 从最近点沿路径向前搜索, 返回第一个距离 >= lookahead 的点
 *
 * 使用 state.x/state.y (车辆当前位置) 作为距离测量参考,
 * 这是标准 Pure Pursuit 的做法 (Bug #21b 修复)
 *
 * 如果路径太短 (所有点到车辆的距离都 < lookahead),
 * 返回路径终点 (让车自然开到终点)
 */
PathPoint PurePursuitTracker::FindLookaheadPoint(const VehicleState& state, size_t closest_idx, double lookahead) const {
    if (path_.empty()) return {};

    // 从最近点开始向前搜索, 找第一个距离 >= lookahead 的点
    for (size_t i = closest_idx; i < path_.size(); ++i) {
        double dx = path_[i].x - state.x;
        double dy = path_[i].y - state.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist >= lookahead) {
            return path_[i];
        }
    }

    // 没找到 → 返回路径终点
    return path_.back();
}

/**
 * @brief Pure Pursuit 控制指令计算 (核心算法, 100Hz 调用)
 *
 * 步骤:
 *   1. 找最近点 → 2. 自适应前视 → 3. 找目标点
 *   → 4. 变换到车辆坐标系 → 5. 计算转向角
 *   → 6. 转向平滑 → 7. 速度控制
 */
ControlCommand PurePursuitTracker::Compute(const VehicleState& state) {
    ControlCommand cmd;
    cmd.timestamp = state.timestamp;

    // 无路径 → 减速并保持方向盘回正
    if (!path_set_ || path_.empty()) {
        cmd.acceleration = -cfg_.kp_velocity * state.linear_velocity;
        cmd.steering_angle = 0.0;
        last_steering_ = 0.0;
        return cmd;
    }

    // 1. 在路径上找到离车辆最近的点
    size_t closest = FindClosestPoint(state.x, state.y);

    // 2. 自适应前视距离: 基础值 + 速度系数
    //    Bug #25 修复: 缩短前视 (base 3.0→1.5, 系数 2.0→1.0, 上限 8.0→4.0)
    //    短前视让弯道响应更快
    double ld = cfg_.lookahead_distance + 1.0 * std::abs(state.linear_velocity);
    ld = std::max(1.0, std::min(4.0, ld));  // 限制在 [1.0, 4.0] 米

    // 3. 找到前视距离处的目标路径点
    PathPoint target = FindLookaheadPoint(state, closest, ld);

    // 4. 将目标点变换到车辆坐标系, 计算横向偏移
    //    旋转公式: local_y = -sin(θ)*dx + cos(θ)*dy
    double dx = target.x - state.x;
    double dy = target.y - state.y;
    double cos_theta = std::cos(state.theta);
    double sin_theta = std::sin(state.theta);
    double local_y = -sin_theta * dx + cos_theta * dy;

    // 5. Pure Pursuit 核心公式:
    //       曲率 κ = 2 * local_y / Ld²
    //       转向角 δ = atan2(κ * wheel_base, 1)
    double curvature = 2.0 * local_y / (ld * ld);
    double raw_steering = std::atan2(curvature * cfg_.wheel_base, 1.0);

    // 6. 转向平滑: 速率限制 (3.0 rad/s, 每帧最大变化 0.03 rad @100Hz)
    //    防止转向角帧间突变导致车辆抖动
    double max_rate = 3.0;
    double max_delta = max_rate * 0.01;  // dt ≈ 0.01s @100Hz
    double delta = raw_steering - last_steering_;
    delta = std::max(-max_delta, std::min(max_delta, delta));
    double steering = last_steering_ + delta;
    last_steering_ = steering;
    // 钳位到最大转向角 [-max_steering, +max_steering]
    cmd.steering_angle = std::max(-cfg_.max_steering,
                                  std::min(cfg_.max_steering, steering));

    // 7. 曲率自适应速度控制:
    //    desired_speed = target_velocity / (1 + 10 * |curvature|)
    //    弯道半径小→曲率大→速度降→安全通行
    //    Bug #25 修复: 弯道减速系数 4.0→10.0, 弯道降速更激进
    double abs_curvature = std::abs(curvature);
    double speed_factor = 1.0 / (1.0 + 10.0 * abs_curvature);
    double desired_speed = cfg_.target_velocity * speed_factor;

    // 8. 速度 P 控制: accel = kp * (desired - current), 限制 ±2.0 m/s²
    double v_error = desired_speed - state.linear_velocity;
    cmd.acceleration = std::max(-2.0, std::min(2.0, cfg_.kp_velocity * v_error));

    return cmd;
}

/**
 * @brief 终点判定: 当前距离 < goal_tolerance_xy
 *
 * 仅判断 XY 距离 (不判断朝向), 因为:
 *   1. 路径终点的朝向可能与用户指定的目标朝向有细微差异
 *   2. 到达后停车即可, 不需精确对齐朝向
 */
bool PurePursuitTracker::IsGoalReached(const VehicleState& state) {
    if (!path_set_ || path_.empty()) return true;

    const auto& goal = path_.back();
    double dx = goal.x - state.x;
    double dy = goal.y - state.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    return dist < cfg_.goal_tolerance_xy;
}
