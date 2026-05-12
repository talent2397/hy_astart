/**
 * @file hybrid_a_star_planner.cpp
 * @brief HybridAStarPlanner — IPlanner 接口实现
 *
 * 适配器角色: 将通用的 IPlanner 接口适配到 HybridAStar 核心搜索引擎。
 * 负责数据类型转换 (MapData → 内部障碍物, VectorVec3d → Path)
 * 和参数传递。
 */

#include "planner/hybrid_a_star_planner.h"
#include <glog/logging.h>

bool HybridAStarPlanner::Init(const PlannerConfig& cfg) {
    config_ = cfg;

    // 创建 HybridAStar 核心搜索器, 传入所有运动学和惩罚参数
    searcher_ = std::make_unique<HybridAStar>(
        cfg.steering_angle, cfg.steering_angle_discrete_num,
        cfg.segment_length, cfg.segment_length_discrete_num,
        cfg.wheel_base, cfg.steering_penalty,
        cfg.reversing_penalty, cfg.steering_change_penalty,
        cfg.shot_distance, cfg.grid_size_phi
    );

    LOG(INFO) << "HybridAStarPlanner initialized. "
              << "steering_angle=" << cfg.steering_angle
              << " wheel_base=" << cfg.wheel_base
              << " segment_length=" << cfg.segment_length;

    return true;
}

void HybridAStarPlanner::LoadMap(const MapData& map) {
    if (!searcher_) return;

    // 计算地图在世界坐标系中的边界
    map_x_lower_ = map.origin_x;
    map_x_upper_ = map.origin_x + map.width * map.resolution;
    map_y_lower_ = map.origin_y;
    map_y_upper_ = map.origin_y + map.height * map.resolution;

    // 初始化搜索空间: 设置两个层级的网格
    //   - STATE_GRID: 状态空间离散化 (粗, ~1m)
    //   - MAP_GRID: 障碍物地图 (细, ~0.1m)
    searcher_->Init(map_x_lower_, map_x_upper_, map_y_lower_, map_y_upper_,
                    config_.state_grid_resolution, map.resolution);

    // 将占用栅格逐格加载到搜索器
    for (int h = 0; h < map.height; ++h) {
        for (int w = 0; w < map.width; ++w) {
            if (map.data[h * map.width + w] > 0) {  // >0 表示被占用
                searcher_->SetObstacle(static_cast<unsigned int>(w), static_cast<unsigned int>(h));
            }
        }
    }

    // 设置车辆碰撞检测用的矩形尺寸: 长 2.0m × 宽 1.0m, 后轴距车尾 0.5m
    // Bug #15 修复: 之前为 4.7×2.0 (过大), 改为与实际小车一致的 2.0×1.0
    searcher_->SetVehicleShape(2.0, 1.0, 0.5);

    LOG(INFO) << "Map loaded: " << map.width << "x" << map.height
              << " res=" << map.resolution
              << " bounds=[" << map_x_lower_ << "," << map_x_upper_
              << "]x[" << map_y_lower_ << "," << map_y_upper_ << "]";
}

bool HybridAStarPlanner::Plan(const Pose2D& start, const Pose2D& goal,
                               const MapData& map, Path& out_path) {
    if (!searcher_) {
        LOG(ERROR) << "Planner not initialized";
        return false;
    }

    // 每次规划前重新加载地图 (支持地图更新, 虽然当前地图是静态的)
    LoadMap(map);

    Vec3d start_state(start.x, start.y, start.theta);
    Vec3d goal_state(goal.x, goal.y, goal.theta);

    LOG(INFO) << "Planning from (" << start.x << "," << start.y << "," << start.theta
              << ") to (" << goal.x << "," << goal.y << "," << goal.theta << ")";

    bool success = searcher_->Search(start_state, goal_state);

    if (success) {
        // 将 Eigen 格式路径 (VectorVec3d) 转换为通用 Path 类型
        VectorVec3d raw_path = searcher_->GetPath();
        out_path.clear();
        out_path.reserve(raw_path.size());

        for (const auto& p : raw_path) {
            PathPoint pt;
            pt.x = p.x();
            pt.y = p.y();
            pt.theta = p.z();
            pt.v_ref = 1.0;  // 默认参考速度 1.0 m/s
            out_path.push_back(pt);
        }

        LOG(INFO) << "Path found: " << out_path.size() << " points, "
                  << "length=" << searcher_->GetPathLength() << "m, "
                  << "visited=" << searcher_->GetVisitedNodesNumber();
    } else {
        LOG(WARNING) << "No path found";
    }

    return success;
}

VectorVec4d HybridAStarPlanner::GetSearchTree() const {
    if (searcher_) return searcher_->GetSearchedTree();
    return {};
}

int HybridAStarPlanner::GetVisitedNodes() const {
    if (searcher_) return searcher_->GetVisitedNodesNumber();
    return 0;
}

void HybridAStarPlanner::Reset() {
    if (searcher_) searcher_->Reset();
}
