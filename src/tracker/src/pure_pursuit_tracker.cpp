#include "tracker/pure_pursuit_tracker.h"
#include <glog/logging.h>
#include <algorithm>
#include <limits>

bool PurePursuitTracker::Init(const TrackerConfig& cfg) {
    cfg_ = cfg;
    LOG(INFO) << "PurePursuitTracker initialized. "
              << "lookahead=" << cfg_.lookahead_distance
              << " target_vel=" << cfg_.target_velocity
              << " wheel_base=" << cfg_.wheel_base
              << " max_steering=" << cfg_.max_steering;
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

size_t PurePursuitTracker::FindClosestPoint(double x, double y) const {
    if (path_.empty()) return 0;

    size_t closest = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < path_.size(); ++i) {
        double dx = path_[i].x - x;
        double dy = path_[i].y - y;
        double dist = dx * dx + dy * dy;  // squared distance
        if (dist < min_dist) {
            min_dist = dist;
            closest = i;
        }
    }
    return closest;
}

PathPoint PurePursuitTracker::FindLookaheadPoint(size_t closest_idx, double lookahead) const {
    if (path_.empty()) return {};

    // 从最近点向前搜索，直到距离 >= lookahead
    // 或者到达路径终点
    double vehicle_x = path_[closest_idx].x;
    double vehicle_y = path_[closest_idx].y;

    // 从 closest_idx 开始，沿 path 向前
    for (size_t i = closest_idx; i < path_.size(); ++i) {
        double dx = path_[i].x - vehicle_x;
        double dy = path_[i].y - vehicle_y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist >= lookahead) {
            return path_[i];
        }
    }

    // 如果没找到（路径太短），返回最后一个点
    return path_.back();
}

ControlCommand PurePursuitTracker::Compute(const VehicleState& state) {
    ControlCommand cmd;
    cmd.timestamp = state.timestamp;

    if (!path_set_ || path_.empty()) {
        // 没有路径，停止
        cmd.acceleration = -cfg_.kp_velocity * state.linear_velocity;
        cmd.steering_angle = 0.0;
        return cmd;
    }

    // 1. 找到路径上最近的点
    size_t closest = FindClosestPoint(state.x, state.y);

    // 2. 找到前视距离处的目标点
    PathPoint target = FindLookaheadPoint(closest, cfg_.lookahead_distance);

    // 3. 计算转向角（Pure Pursuit 核心公式）
    // 将目标点转换到车辆坐标系
    double dx = target.x - state.x;
    double dy = target.y - state.y;

    // 旋转到车辆坐标系
    double cos_theta = std::cos(state.theta);
    double sin_theta = std::sin(state.theta);
    double local_x = cos_theta * dx + sin_theta * dy;
    double local_y = -sin_theta * dx + cos_theta * dy;

    // Pure Pursuit 公式: δ = atan2(2 * L * local_y / Ld^2)
    double ld = cfg_.lookahead_distance;
    double curvature = 2.0 * local_y / (ld * ld);
    double steering = std::atan2(curvature * cfg_.wheel_base, 1.0);

    // 限制转向角
    cmd.steering_angle = std::max(-cfg_.max_steering,
                                  std::min(cfg_.max_steering, steering));

    // 4. 速度控制（简单 P 控制）
    double v_error = cfg_.target_velocity - state.linear_velocity;
    cmd.acceleration = cfg_.kp_velocity * v_error;

    return cmd;
}

bool PurePursuitTracker::IsGoalReached(const VehicleState& state) {
    if (!path_set_ || path_.empty()) return true;

    const auto& goal = path_.back();
    double dx = goal.x - state.x;
    double dy = goal.y - state.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    return dist < cfg_.goal_tolerance_xy;
}
