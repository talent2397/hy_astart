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

PathPoint PurePursuitTracker::FindLookaheadPoint(const VehicleState& state, size_t closest_idx, double lookahead) const {
    if (path_.empty()) return {};

    // 从车辆当前位置向前搜索，找距离 >= lookahead 的路径点
    for (size_t i = closest_idx; i < path_.size(); ++i) {
        double dx = path_[i].x - state.x;
        double dy = path_[i].y - state.y;
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
        cmd.acceleration = -cfg_.kp_velocity * state.linear_velocity;
        cmd.steering_angle = 0.0;
        last_steering_ = 0.0;
        return cmd;
    }

    // 1. 找到路径上最近的点
    size_t closest = FindClosestPoint(state.x, state.y);

    // 2. 自适应前视距离: 短前视 = 转弯更灵敏
    double ld = cfg_.lookahead_distance + 1.0 * std::abs(state.linear_velocity);
    ld = std::max(1.0, std::min(4.0, ld));

    // 3. 找到前视距离处的目标点
    PathPoint target = FindLookaheadPoint(state, closest, ld);

    // 4. 变换到车辆坐标系
    double dx = target.x - state.x;
    double dy = target.y - state.y;
    double cos_theta = std::cos(state.theta);
    double sin_theta = std::sin(state.theta);
    double local_y = -sin_theta * dx + cos_theta * dy;

    // 5. Pure Pursuit 转向角
    double curvature = 2.0 * local_y / (ld * ld);
    double raw_steering = std::atan2(curvature * cfg_.wheel_base, 1.0);

    // 6. 转向速率限制 (防突变, 3.0 rad/s 足够快速响应)
    double max_rate = 3.0;
    double max_delta = max_rate * 0.01;  // dt=0.01s @100Hz
    double delta = raw_steering - last_steering_;
    delta = std::max(-max_delta, std::min(max_delta, delta));
    double steering = last_steering_ + delta;
    last_steering_ = steering;
    cmd.steering_angle = std::max(-cfg_.max_steering,
                                  std::min(cfg_.max_steering, steering));

    // 7. 曲率自适应速度: 弯道大幅减速
    double abs_curvature = std::abs(curvature);
    double speed_factor = 1.0 / (1.0 + 10.0 * abs_curvature);
    double desired_speed = cfg_.target_velocity * speed_factor;

    // 8. 速度 P 控制 (限制加速度)
    double v_error = desired_speed - state.linear_velocity;
    cmd.acceleration = std::max(-2.0, std::min(2.0, cfg_.kp_velocity * v_error));

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
