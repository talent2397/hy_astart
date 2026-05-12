/**
 * @file hybrid_a_star.cpp
 * @brief Hybrid A* 核心搜索算法实现
 *
 * 算法流程概览:
 *   1. 初始化: 建立 3D 状态网格 (x, y, θ), 加载障碍物地图
 *   2. 将起点加入 openset (multimap, 按 f 值排序)
 *   3. 循环:
 *      a. 从 openset 取出 f 值最小的节点 (最佳优先)
 *      b. 如果节点距终点 < shot_distance, 尝试 RS 曲线直连 → 成功则搜索结束
 *      c. 生成邻居节点 (自行车运动模型, 前进+后退, 各转向角)
 *      d. 碰撞检测 (矩形车辆模型 + Bresenham 线段检测)
 *      e. 计算 g 代价 (含转向/倒车惩罚) 和 h 启发值
 *      f. 离散化到 3D 网格: 同格保留 g 最小的节点
 *      g. 将有效邻居加入 openset
 *   4. 搜索成功: 反向回溯 parent_node_ 链表提取路径
 *   5. 搜索失败: openset 空 或 超过最大迭代次数 (50000)
 *
 * 关键设计决策:
 *   - 状态空间网格 (STATE_GRID) 比地图网格 (MAP_GRID) 粗 ~10 倍,
 *     只有不同 (x, y, θ) 网格的节点才被视为不同状态
 *   - RS 曲线捷径大幅加速收敛: 距终点 5m 内直接尝试解析解
 *   - 碰撞检测沿运动路径细分 (segment_length_discrete_num 段),
 *     确保不遗漏障碍物
 */

#include "planner/hybrid_a_star.h"
#include "planner/trajectory_optimizer.h"
#include "common/timer.h"

#include <glog/logging.h>
#include <iostream>

// ============================================================
// 构造函数: 保存运动学参数, 初始化 RS 曲线计算器
// ============================================================
HybridAStar::HybridAStar(double steering_angle, int steering_angle_discrete_num, double segment_length,
                         int segment_length_discrete_num, double wheel_base, double steering_penalty,
                         double reversing_penalty, double steering_change_penalty, double shot_distance,
                         int grid_size_phi) {
    wheel_base_ = wheel_base;
    segment_length_ = segment_length;
    steering_radian_ = steering_angle * M_PI / 180.0;       // 度 → 弧度
    steering_discrete_num_ = steering_angle_discrete_num;
    steering_radian_step_size_ = steering_radian_ / steering_discrete_num_;  // 每档转向角增量
    move_step_size_ = segment_length / segment_length_discrete_num;          // 每小步距离
    segment_length_discrete_num_ = static_cast<int>(segment_length_discrete_num);
    steering_penalty_ = steering_penalty;
    steering_change_penalty_ = steering_change_penalty;
    reversing_penalty_ = reversing_penalty;
    shot_distance_ = shot_distance;

    // 验证步长可被整除 (否则碰撞检测可能漏检)
    CHECK_EQ(static_cast<float>(segment_length_discrete_num_ * move_step_size_), static_cast<float>(segment_length_))
        << "The segment length must be divisible by the step size. segment_length: "
        << segment_length_ << " | step_size: " << move_step_size_;

    // RS 曲线计算器: 转弯半径 = wheel_base / tan(max_steering)
    rs_path_ptr_ = std::make_shared<RSPath>(wheel_base_ / std::tan(steering_radian_));
    tie_breaker_ = 1.0 + 1e-3;  // 微小增量打破平局, 使算法偏向 g 值更小的节点

    STATE_GRID_SIZE_PHI_ = grid_size_phi;
    ANGULAR_RESOLUTION_ = 360.0 / STATE_GRID_SIZE_PHI_ * M_PI / 180.0;  // 如 72 → 5° = 0.087rad
}

HybridAStar::~HybridAStar() {
    ReleaseMemory();
}

// ============================================================
// Init: 建立两个层级的网格
//   STATE_GRID: 状态空间离散化 (x, y, θ), 避免重复搜索相同状态
//   MAP_GRID:   障碍物地图, 高分辨率
// state_node_map_: 3D 指针网格, 每个 grid cell 最多存一个 StateNode*
// ============================================================
void HybridAStar::Init(double x_lower, double x_upper, double y_lower, double y_upper,
                       double state_grid_resolution, double map_grid_resolution) {
    map_x_lower_ = x_lower;
    map_x_upper_ = x_upper;
    map_y_lower_ = y_lower;
    map_y_upper_ = y_upper;
    STATE_GRID_RESOLUTION_ = state_grid_resolution;
    MAP_GRID_RESOLUTION_ = map_grid_resolution;

    STATE_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / STATE_GRID_RESOLUTION_);
    STATE_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / STATE_GRID_RESOLUTION_);

    MAP_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / MAP_GRID_RESOLUTION_);
    MAP_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / MAP_GRID_RESOLUTION_);

    // 分配障碍物地图 (全部初始化为 0 = 空闲)
    if (map_data_) {
        delete[] map_data_;
        map_data_ = nullptr;
    }
    map_data_ = new uint8_t[MAP_GRID_SIZE_X_ * MAP_GRID_SIZE_Y_]();

    // 释放旧的 3D 状态网格
    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            if (state_node_map_[i] == nullptr) continue;
            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr) continue;
                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    delete state_node_map_[i][j][k];
                    state_node_map_[i][j][k] = nullptr;
                }
                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }
            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr;
        }
        delete[] state_node_map_;
        state_node_map_ = nullptr;
    }

    // 分配新的 3D 状态网格 (全部初始化为 nullptr)
    state_node_map_ = new StateNode::Ptr**[STATE_GRID_SIZE_X_];
    for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
        state_node_map_[i] = new StateNode::Ptr*[STATE_GRID_SIZE_Y_];
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
            state_node_map_[i][j] = new StateNode::Ptr[STATE_GRID_SIZE_PHI_];
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                state_node_map_[i][j][k] = nullptr;
            }
        }
    }
}

// ============================================================
// 碰撞检测
// ============================================================

/**
 * @brief Bresenham 线段检测: 检查从 (x0,y0) 到 (x1,y1) 的线段是否穿过障碍物
 *
 * 经典 Bresenham 直线算法, 逐像素检测。
 * 处理 steep (斜率 > 1) 的情况, 交换 x/y 确保每次沿 x 轴步进。
 *
 * @return true 表示线段上没有障碍物 (可以通过)
 */
bool HybridAStar::LineCheck(double x0, double y0, double x1, double y1) {
    // 陡峭线段 (|dy| > |dx|): 交换 x,y 确保沿 x 轴步进
    bool steep = (std::abs(y1 - y0) > std::abs(x1 - x0));
    if (steep) { std::swap(x0, y0); std::swap(y1, x1); }
    if (x0 > x1) { std::swap(x0, x1); std::swap(y0, y1); }  // 保证 x 从左到右

    auto delta_x = x1 - x0;
    auto delta_y = std::abs(y1 - y0);
    auto delta_error = delta_y / delta_x;
    decltype(delta_x) error = 0;
    decltype(delta_x) y_step;
    auto yk = y0;
    y_step = (y0 < y1) ? 1 : -1;

    auto N = static_cast<unsigned int>(x1 - x0);
    for (unsigned int i = 0; i < N; ++i) {
        if (steep) {
            if (HasObstacle(Vec2i(yk, x0 + i * 1.0)) ||
                BeyondBoundary(Vec2d(yk * MAP_GRID_RESOLUTION_ + map_x_lower_,
                                     (x0 + i) * MAP_GRID_RESOLUTION_ + map_y_lower_)))
                return false;  // 碰到障碍物或出界
        } else {
            if (HasObstacle(Vec2i(x0 + i * 1.0, yk)) ||
                BeyondBoundary(Vec2d((x0 + i) * MAP_GRID_RESOLUTION_ + map_x_lower_,
                                     yk * MAP_GRID_RESOLUTION_ + map_y_lower_)))
                return false;
        }
        error += delta_error;
        if (error >= 0.5) { yk += y_step; error -= 1.0; }
    }
    return true;
}

/**
 * @brief 矩形车辆碰撞检测
 *
 * 将车辆矩形 (4 个顶点) 根据当前位姿 (x, y, θ) 变换到世界坐标,
 * 然后检测四条边是否穿过障碍物。
 *
 * 车辆矩形定义:
 *   (0) 左后    (1) 右后
 *    *-----------*
 *    |           |
 *    |  车辆中心  |
 *    |           |
 *    *-----------*
 *   (3) 左前    (2) 右前
 *
 * @return true 表示无碰撞
 */
bool HybridAStar::CheckCollision(const double& x, const double& y, const double& theta) {
    // 2D 旋转矩阵
    Mat2d R;
    R << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);

    // 将车辆 4 个顶点从车辆坐标系变换到世界坐标系
    MatXd transformed_vehicle_shape;
    transformed_vehicle_shape.resize(8, 1);
    for (unsigned int i = 0; i < 4u; ++i) {
        transformed_vehicle_shape.block<2, 1>(i * 2, 0)
            = R * vehicle_shape_.block<2, 1>(i * 2, 0) + Vec2d(x, y);
    }

    // 将顶点转为地图网格索引
    Vec2i pt0 = Coordinate2MapGridIndex(transformed_vehicle_shape.block<2, 1>(0, 0));
    Vec2i pt1 = Coordinate2MapGridIndex(transformed_vehicle_shape.block<2, 1>(2, 0));
    Vec2i pt2 = Coordinate2MapGridIndex(transformed_vehicle_shape.block<2, 1>(4, 0));
    Vec2i pt3 = Coordinate2MapGridIndex(transformed_vehicle_shape.block<2, 1>(6, 0));

    // Bresenham 检测四条边
    if (!LineCheck(pt1.x(), pt1.y(), pt0.x(), pt0.y())) return false;  // 后边
    if (!LineCheck(pt2.x(), pt2.y(), pt1.x(), pt1.y())) return false;  // 右边
    if (!LineCheck(pt3.x(), pt3.y(), pt2.x(), pt2.y())) return false;  // 前边
    if (!LineCheck(pt0.x(), pt0.y(), pt3.x(), pt3.y())) return false;  // 左边

    return true;
}

// ---- 障碍物查询 ----

bool HybridAStar::HasObstacle(const int grid_index_x, const int grid_index_y) const {
    return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_ &&
            grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_ &&
            (map_data_[grid_index_y * MAP_GRID_SIZE_X_ + grid_index_x] == 1));
}

bool HybridAStar::HasObstacle(const Vec2i& grid_index) const {
    return HasObstacle(grid_index[0], grid_index[1]);
}

void HybridAStar::SetObstacle(unsigned int x, unsigned int y) {
    if (x >= static_cast<unsigned int>(MAP_GRID_SIZE_X_) ||
        y >= static_cast<unsigned int>(MAP_GRID_SIZE_Y_)) return;
    map_data_[x + y * MAP_GRID_SIZE_X_] = 1;  // 注意: 列优先 (x + y*width)
}

void HybridAStar::SetObstacle(double pt_x, double pt_y) {
    if (pt_x < map_x_lower_ || pt_x > map_x_upper_ ||
        pt_y < map_y_lower_ || pt_y > map_y_upper_) return;
    int gx = static_cast<int>((pt_x - map_x_lower_) / MAP_GRID_RESOLUTION_);
    int gy = static_cast<int>((pt_y - map_y_lower_) / MAP_GRID_RESOLUTION_);
    map_data_[gx + gy * MAP_GRID_SIZE_X_] = 1;
}

/**
 * @brief 设置车辆碰撞矩形 + 离散化轮廓
 *
 * 车辆矩形 4 个顶点 (相对于后轴中心):
 *   (0) (-rear_axle_dist,  width/2)  — 左后
 *   (1) (length-rear_axle_dist,  width/2)  — 右后
 *   (2) (length-rear_axle_dist, -width/2)  — 右前
 *   (3) (-rear_axle_dist, -width/2)  — 左前
 *
 * 同时离散化四条边上的点 (vehicle_shape_discrete_),
 * 用于更精细的碰撞检测 (如轨迹优化器使用)
 */
void HybridAStar::SetVehicleShape(double length, double width, double rear_axle_dist) {
    vehicle_shape_.resize(8);
    vehicle_shape_.block<2, 1>(0, 0) = Vec2d(-rear_axle_dist, width / 2);
    vehicle_shape_.block<2, 1>(2, 0) = Vec2d(length - rear_axle_dist, width / 2);
    vehicle_shape_.block<2, 1>(4, 0) = Vec2d(length - rear_axle_dist, -width / 2);
    vehicle_shape_.block<2, 1>(6, 0) = Vec2d(-rear_axle_dist, -width / 2);

    const double step = move_step_size_;
    const auto N_len = static_cast<unsigned int>(length / step);  // 长边采样点数
    const auto N_wid = static_cast<unsigned int>(width / step);   // 宽边采样点数
    vehicle_shape_discrete_.resize(2, (N_len + N_wid) * 2u);

    // 离散化两条长边
    const Vec2d e0_norm = (vehicle_shape_.block<2, 1>(2, 0) - vehicle_shape_.block<2, 1>(0, 0)).normalized();
    for (unsigned int i = 0; i < N_len; ++i) {
        vehicle_shape_discrete_.block<2, 1>(0, i + N_len) =
            vehicle_shape_.block<2, 1>(4, 0) - e0_norm * i * step;
        vehicle_shape_discrete_.block<2, 1>(0, i) =
            vehicle_shape_.block<2, 1>(0, 0) + e0_norm * i * step;
    }

    // 离散化两条宽边
    const Vec2d e1_norm = (vehicle_shape_.block<2, 1>(4, 0) - vehicle_shape_.block<2, 1>(2, 0)).normalized();
    for (unsigned int i = 0; i < N_wid; ++i) {
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_len) + i) =
            vehicle_shape_.block<2, 1>(2, 0) + e1_norm * i * step;
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_len) + i + N_wid) =
            vehicle_shape_.block<2, 1>(6, 0) - e1_norm * i * step;
    }
}

// ============================================================
// 坐标转换
// ============================================================

/// 世界坐标 → 地图网格索引 (取整)
Vec2d HybridAStar::CoordinateRounding(const Vec2d& pt) const {
    return MapGridIndex2Coordinate(Coordinate2MapGridIndex(pt));
}

/// 地图网格索引 → 世界坐标 (返回栅格中心点)
Vec2d HybridAStar::MapGridIndex2Coordinate(const Vec2i& grid_index) const {
    Vec2d pt;
    pt.x() = ((double)grid_index[0] + 0.5) * MAP_GRID_RESOLUTION_ + map_x_lower_;
    pt.y() = ((double)grid_index[1] + 0.5) * MAP_GRID_RESOLUTION_ + map_y_lower_;
    return pt;
}

/// 连续状态 (x, y, θ) → 3D 状态网格索引
/// θ 映射: 将 (-π, π] 映射到 [0, STATE_GRID_SIZE_PHI_)
Vec3i HybridAStar::State2Index(const Vec3d& state) const {
    Vec3i index;
    index[0] = std::min(std::max(int((state[0] - map_x_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_X_ - 1);
    index[1] = std::min(std::max(int((state[1] - map_y_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_Y_ - 1);
    index[2] = std::min(std::max(int((state[2] - (-M_PI)) / ANGULAR_RESOLUTION_), 0), STATE_GRID_SIZE_PHI_ - 1);
    return index;
}

/// 世界坐标 → 地图网格索引
Vec2i HybridAStar::Coordinate2MapGridIndex(const Vec2d& pt) const {
    Vec2i grid_index;
    grid_index[0] = int((pt[0] - map_x_lower_) / MAP_GRID_RESOLUTION_);
    grid_index[1] = int((pt[1] - map_y_lower_) / MAP_GRID_RESOLUTION_);
    return grid_index;
}

// ============================================================
// 邻居生成 & 自行车运动模型
// ============================================================

/**
 * @brief 从当前节点生成所有邻居节点
 *
 * 对每个可能的转向档位 i ∈ [-N, N]:
 *   - 前进: 用 DynamicModel 模拟 segment_length 米 (细分 segment_length_discrete_num 步)
 *   - 后退: 同上, 步长为负
 *
 * 沿途每小步做碰撞检测, 任何一步碰撞则该方向邻居无效。
 * 最终状态离散化到 3D 网格, 出界或在障碍物中的邻居被丢弃。
 */
void HybridAStar::GetNeighborNodes(const StateNode::Ptr& curr_node_ptr,
                                   std::vector<StateNode::Ptr>& neighbor_nodes) {
    neighbor_nodes.clear();

    // 遍历所有可能的转向角: -max, -max+step, ..., 0, ..., max-step, max
    for (int i = -steering_discrete_num_; i <= steering_discrete_num_; ++i) {
        VectorVec3d intermediate_state;
        bool has_obstacle = false;
        double x = curr_node_ptr->state_.x();
        double y = curr_node_ptr->state_.y();
        double theta = curr_node_ptr->state_.z();
        const double phi = i * steering_radian_step_size_;  // 本次转向角

        // ---- 前进方向 ----
        for (int j = 1; j <= segment_length_discrete_num_; j++) {
            DynamicModel(move_step_size_, phi, x, y, theta);
            intermediate_state.emplace_back(Vec3d(x, y, theta));
            if (!CheckCollision(x, y, theta)) { has_obstacle = true; break; }
        }

        Vec3i grid_index = State2Index(intermediate_state.back());
        if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) {
            auto node = new StateNode(grid_index);
            node->intermediate_states_ = intermediate_state;
            node->state_ = intermediate_state.back();
            node->steering_grade_ = i;
            node->direction_ = StateNode::FORWARD;
            neighbor_nodes.push_back(node);
        }

        // ---- 后退方向 ----
        has_obstacle = false;
        intermediate_state.clear();
        x = curr_node_ptr->state_.x();
        y = curr_node_ptr->state_.y();
        theta = curr_node_ptr->state_.z();
        for (int j = 1; j <= segment_length_discrete_num_; j++) {
            DynamicModel(-move_step_size_, phi, x, y, theta);  // 负步长 = 倒车
            intermediate_state.emplace_back(Vec3d(x, y, theta));
            if (!CheckCollision(x, y, theta)) { has_obstacle = true; break; }
        }

        if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) {
            grid_index = State2Index(intermediate_state.back());
            auto node = new StateNode(grid_index);
            node->intermediate_states_ = intermediate_state;
            node->state_ = intermediate_state.back();
            node->steering_grade_ = i;
            node->direction_ = StateNode::BACKWARD;
            neighbor_nodes.push_back(node);
        }
    }
}

/**
 * @brief 自行车运动模型 (离散时间欧拉积分)
 *
 * 状态更新方程:
 *   x(t+dt) = x(t) + v * cos(θ) * dt
 *   y(t+dt) = y(t) + v * sin(θ) * dt
 *   θ(t+dt) = θ(t) + v/L * tan(φ) * dt
 *
 * 其中:
 *   v = step_size (正=前进, 负=后退)
 *   φ = 前轮转向角
 *   L = 轴距 wheel_base_
 */
void HybridAStar::DynamicModel(const double& step_size, const double& phi,
                               double& x, double& y, double& theta) const {
    x = x + step_size * std::cos(theta);
    y = y + step_size * std::sin(theta);
    theta = Mod2Pi(theta + step_size / wheel_base_ * std::tan(phi));
}

/// 角度归一化到 (-π, π]
double HybridAStar::Mod2Pi(const double& x) {
    double v = fmod(x, 2 * M_PI);
    if (v < -M_PI) v += 2.0 * M_PI;
    else if (v > M_PI) v -= 2.0 * M_PI;
    return v;
}

/// 检查点是否超出地图边界
bool HybridAStar::BeyondBoundary(const Vec2d& pt) const {
    return pt.x() < map_x_lower_ || pt.x() > map_x_upper_ ||
           pt.y() < map_y_lower_ || pt.y() > map_y_upper_;
}

// ============================================================
// A* 代价函数
// ============================================================

/**
 * @brief 启发式函数 h(n): 估计从当前节点到终点的代价
 *
 * 策略:
 *   近距离 (< 3*shot_distance): 用 RS 曲线最短距离 (考虑运动学约束, 更准确)
 *   远距离:                    用 L1 (曼哈顿) 距离 (计算快)
 *
 * RS 距离需要除以 turning_radius 来归一化
 */
double HybridAStar::ComputeH(const StateNode::Ptr& current_node_ptr,
                             const StateNode::Ptr& terminal_node_ptr) {
    double h = (current_node_ptr->state_.head(2) - terminal_node_ptr->state_.head(2)).lpNorm<1>();

    if (h < 3.0 * shot_distance_) {
        h = rs_path_ptr_->Distance(current_node_ptr->state_.x(), current_node_ptr->state_.y(),
                                   current_node_ptr->state_.z(),
                                   terminal_node_ptr->state_.x(), terminal_node_ptr->state_.y(),
                                   terminal_node_ptr->state_.z());
    }
    return h;
}

/**
 * @brief 实际代价 g(n): 从起点到当前节点的累积代价
 *
 * 代价 = segment_length * 惩罚系数
 *
 * 惩罚规则:
 *   - 直线前进 (steering_grade=0, FORWARD): 1.0×
 *   - 转向前进 (steering_grade≠0, FORWARD): steering_penalty×
 *   - 直线倒车 (steering_grade=0, BACKWARD): reversing_penalty×
 *   - 转向倒车 (steering_grade≠0, BACKWARD): steering_penalty × reversing_penalty×
 *   - 转向变化 (grade 变化): 额外 × steering_change_penalty
 *
 * 鼓励: 直线前进 > 转向前进 > 直线倒车 > 转向倒车
 */
double HybridAStar::ComputeG(const StateNode::Ptr& current_node_ptr,
                             const StateNode::Ptr& neighbor_node_ptr) const {
    double g;
    if (neighbor_node_ptr->direction_ == StateNode::FORWARD) {
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {
            // 转向角变化
            g = (neighbor_node_ptr->steering_grade_ == 0) ?
                segment_length_ * steering_change_penalty_ :
                segment_length_ * steering_change_penalty_ * steering_penalty_;
        } else {
            // 转向角不变
            g = (neighbor_node_ptr->steering_grade_ == 0) ?
                segment_length_ :
                segment_length_ * steering_penalty_;
        }
    } else {
        // 倒车: 额外乘以 reversing_penalty
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {
            g = (neighbor_node_ptr->steering_grade_ == 0) ?
                segment_length_ * steering_change_penalty_ * reversing_penalty_ :
                segment_length_ * steering_change_penalty_ * steering_penalty_ * reversing_penalty_;
        } else {
            g = (neighbor_node_ptr->steering_grade_ == 0) ?
                segment_length_ * reversing_penalty_ :
                segment_length_ * steering_penalty_ * reversing_penalty_;
        }
    }
    return g;
}

// ============================================================
// 主搜索循环 — Hybrid A* 核心
// ============================================================

bool HybridAStar::Search(const Vec3d& start_state, const Vec3d& goal_state) {
    Timer search_timer;
    double neighbor_time = 0.0, compute_h_time = 0.0;

    const Vec3i start_grid_index = State2Index(start_state);
    const Vec3i goal_grid_index = State2Index(goal_state);

    VLOG(1) << "Search start: state=(" << start_state.transpose()
            << ") grid=(" << start_grid_index.transpose() << ")"
            << " goal=(" << goal_state.transpose()
            << ") grid=(" << goal_grid_index.transpose() << ")"
            << " grid_sizes=" << STATE_GRID_SIZE_X_ << "x" << STATE_GRID_SIZE_Y_
            << "x" << STATE_GRID_SIZE_PHI_;

    // 创建终点节点 (不是搜索目标, 是 RS 捷径的目标)
    auto goal_node_ptr = new StateNode(goal_grid_index);
    goal_node_ptr->state_ = goal_state;
    goal_node_ptr->direction_ = StateNode::NO;
    goal_node_ptr->steering_grade_ = 0;

    // 创建起点节点
    auto start_node_ptr = new StateNode(start_grid_index);
    start_node_ptr->state_ = start_state;
    start_node_ptr->steering_grade_ = 0;
    start_node_ptr->direction_ = StateNode::NO;
    start_node_ptr->node_status_ = StateNode::IN_OPENSET;
    start_node_ptr->intermediate_states_.emplace_back(start_state);
    start_node_ptr->g_cost_ = 0.0;
    start_node_ptr->f_cost_ = ComputeH(start_node_ptr, goal_node_ptr);  // f = g + h, g=0

    // 将起点和终点注册到 3D 网格
    state_node_map_[start_grid_index.x()][start_grid_index.y()][start_grid_index.z()] = start_node_ptr;
    state_node_map_[goal_grid_index.x()][goal_grid_index.y()][goal_grid_index.z()] = goal_node_ptr;

    // 初始化 openset
    openset_.clear();
    openset_.insert(std::make_pair(0, start_node_ptr));  // f 值作为 key

    std::vector<StateNode::Ptr> neighbor_nodes_ptr;
    StateNode::Ptr current_node_ptr;
    StateNode::Ptr neighbor_node_ptr;

    int count = 0;
    // ========== A* 主循环 ==========
    while (!openset_.empty()) {
        // 1. 取出 f 值最小的节点 (multimap 保证 begin() 是最小 key)
        current_node_ptr = openset_.begin()->second;
        current_node_ptr->node_status_ = StateNode::IN_CLOSESET;
        openset_.erase(openset_.begin());  // 从 openset 移除

        // 2. RS 捷径检测: 距终点 < shot_distance 时尝试直连
        double dist_to_goal = (current_node_ptr->state_.head(2) - goal_node_ptr->state_.head(2)).norm();
        if (dist_to_goal <= shot_distance_) {
            double rs_length = 0.0;
            if (AnalyticExpansions(current_node_ptr, goal_node_ptr, rs_length)) {
                // RS 曲线无碰撞 → 搜索成功!
                terminal_node_ptr_ = goal_node_ptr;
                // 计算路径总长度: 回溯所有节点, 每个节点贡献 segment_length,
                // 最后一段 (当前节点→终点) 用 RS 长度替换
                StateNode::Ptr grid_node_ptr = terminal_node_ptr_->parent_node_;
                while (grid_node_ptr != nullptr) {
                    grid_node_ptr = grid_node_ptr->parent_node_;
                    path_length_ += segment_length_;
                }
                path_length_ = path_length_ - segment_length_ + rs_length;

                LOG(INFO) << "Search succeeded. Time: " << search_timer.End()
                          << "ms, path length: " << path_length_
                          << "m, visited: " << visited_node_number_;

                check_collision_use_time = 0.0;
                num_check_collision = 0.0;
                return true;
            }
        }

        // 3. 生成邻居节点 (前进+后退, 各转向角)
        Timer t_neighbor;
        GetNeighborNodes(current_node_ptr, neighbor_nodes_ptr);
        neighbor_time += t_neighbor.End();

        // 4. 对每个邻居:
        //    a. 计算 g 代价
        //    b. 计算 h 启发值
        //    c. 离散化 → 查 3D 网格:
        //       - nullptr:    新节点, 加入 openset
        //       - IN_OPENSET: 如果新 g 更小则更新
        //       - IN_CLOSESET: 丢弃 (已扩展过, 且新 g 不可能更小)
        for (unsigned int i = 0; i < neighbor_nodes_ptr.size(); ++i) {
            neighbor_node_ptr = neighbor_nodes_ptr[i];

            const double neighbor_edge_cost = ComputeG(current_node_ptr, neighbor_node_ptr);

            Timer t_h;
            const double current_h = ComputeH(current_node_ptr, goal_node_ptr) * tie_breaker_;
            compute_h_time += t_h.End();

            const Vec3i& index = neighbor_node_ptr->grid_index_;

            if (state_node_map_[index.x()][index.y()][index.z()] == nullptr) {
                // 新节点: 设置代价并加入 openset
                neighbor_node_ptr->g_cost_ = current_node_ptr->g_cost_ + neighbor_edge_cost;
                neighbor_node_ptr->parent_node_ = current_node_ptr;
                neighbor_node_ptr->node_status_ = StateNode::IN_OPENSET;
                neighbor_node_ptr->f_cost_ = neighbor_node_ptr->g_cost_ + current_h;
                openset_.insert(std::make_pair(neighbor_node_ptr->f_cost_, neighbor_node_ptr));
                state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;
                continue;
            } else if (state_node_map_[index.x()][index.y()][index.z()]->node_status_ == StateNode::IN_OPENSET) {
                // 已在 openset: 检查新路径是否更优 (更小的 g)
                double g_cost_temp = current_node_ptr->g_cost_ + neighbor_edge_cost;
                if (state_node_map_[index.x()][index.y()][index.z()]->g_cost_ > g_cost_temp) {
                    // 更优: 更新代价和父节点
                    neighbor_node_ptr->g_cost_ = g_cost_temp;
                    neighbor_node_ptr->f_cost_ = g_cost_temp + current_h;
                    neighbor_node_ptr->parent_node_ = current_node_ptr;
                    neighbor_node_ptr->node_status_ = StateNode::IN_OPENSET;
                    state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;
                } else {
                    delete neighbor_node_ptr;  // 更差: 丢弃
                }
                continue;
            } else if (state_node_map_[index.x()][index.y()][index.z()]->node_status_ == StateNode::IN_CLOSESET) {
                // 已在 closeset: 丢弃 (已扩展过)
                delete neighbor_node_ptr;
                continue;
            }
        }

        // 5. 迭代次数检查: 超过 50000 次视为搜索失败
        count++;
        if (count > 50000) {
            LOG(WARNING) << "Search failed: exceeded max iterations";
            return false;
        }
    }

    // openset 空: 搜索完所有可达状态, 无路可走
    LOG(WARNING) << "Search failed: openset empty after " << count
                 << " iterations, visited=" << visited_node_number_
                 << " total_collision_checks=" << num_check_collision
                 << " collision_time_ms=" << check_collision_use_time;
    return false;
}

// ============================================================
// 路径提取
// ============================================================

/// 获取搜索树 (所有展开过的边) — 用于 RViz 可视化调试
VectorVec4d HybridAStar::GetSearchedTree() {
    VectorVec4d tree;
    Vec4d point_pair;
    visited_node_number_ = 0;

    for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                if (state_node_map_[i][j][k] == nullptr || state_node_map_[i][j][k]->parent_node_ == nullptr)
                    continue;

                // 1) 节点内部: intermediate_states_ 相邻点之间连线 (模拟路径内的细分)
                const unsigned int n_states = state_node_map_[i][j][k]->intermediate_states_.size() - 1;
                for (unsigned int l = 0; l < n_states; ++l) {
                    point_pair.head(2) = state_node_map_[i][j][k]->intermediate_states_[l].head(2);
                    point_pair.tail(2) = state_node_map_[i][j][k]->intermediate_states_[l + 1].head(2);
                    tree.emplace_back(point_pair);
                }

                // 2) 节点间: 当前节点起点 → 父节点状态 (连接父子节点)
                point_pair.head(2) = state_node_map_[i][j][k]->intermediate_states_[0].head(2);
                point_pair.tail(2) = state_node_map_[i][j][k]->parent_node_->state_.head(2);
                tree.emplace_back(point_pair);
                visited_node_number_++;
            }
        }
    }
    return tree;
}

/// 从终点回溯 parent_node_ 链表, 提取完整的路径点序列
VectorVec3d HybridAStar::GetPath() const {
    VectorVec3d path;
    std::vector<StateNode::Ptr> temp_nodes;

    // 从终点开始回溯
    StateNode::Ptr state_grid_node_ptr = terminal_node_ptr_;
    while (state_grid_node_ptr != nullptr) {
        temp_nodes.emplace_back(state_grid_node_ptr);
        state_grid_node_ptr = state_grid_node_ptr->parent_node_;
    }

    // 反转为起点→终点顺序, 拼接中间状态
    std::reverse(temp_nodes.begin(), temp_nodes.end());
    for (const auto& node : temp_nodes) {
        path.insert(path.end(), node->intermediate_states_.begin(), node->intermediate_states_.end());
    }
    return path;
}

double HybridAStar::GetPathLength() const { return path_length_; }

/// 重置搜索状态 (清理所有节点的 parent 和 status, 但不释放内存)
void HybridAStar::Reset() {
    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            if (state_node_map_[i] == nullptr) continue;
            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr) continue;
                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    delete state_node_map_[i][j][k];
                    state_node_map_[i][j][k] = nullptr;
                }
            }
        }
    }
    path_length_ = 0.0;
    terminal_node_ptr_ = nullptr;
}

/**
 * @brief RS 曲线捷径: 尝试用 Reeds-Shepp 曲线从当前节点直连到终点
 *
 * 流程:
 *   1. 计算从当前状态到终点状态的 RS 曲线
 *   2. 以 move_step_size_ 步长离散化 RS 曲线
 *   3. 逐点检测碰撞
 *   4. 全部无碰撞 → 设置终点节点的 intermediate_states_ 为 RS 路径,
 *      设置 parent_node_ 为当前节点 → 路径回溯时可拼接
 *
 * 移除 RS 路径的第一个点 (与当前状态重合), 避免路径中出现重复点
 */
bool HybridAStar::AnalyticExpansions(const StateNode::Ptr& current_node_ptr,
                                     const StateNode::Ptr& goal_node_ptr, double& length) {
    VectorVec3d rs_path_poses = rs_path_ptr_->GetRSPath(current_node_ptr->state_,
                                                        goal_node_ptr->state_,
                                                        move_step_size_, length);

    // 碰撞检测: RS 曲线上的每个采样点
    for (const auto& pose : rs_path_poses) {
        if (BeyondBoundary(pose.head(2)) || !CheckCollision(pose.x(), pose.y(), pose.z())) {
            return false;  // 碰撞或出界 → RS 捷径不可行
        }
    }

    // RS 路径有效: 挂接到终点节点
    goal_node_ptr->intermediate_states_ = rs_path_poses;
    goal_node_ptr->parent_node_ = current_node_ptr;

    // 移除第一个点 (与当前节点状态重合)
    auto begin = goal_node_ptr->intermediate_states_.begin();
    goal_node_ptr->intermediate_states_.erase(begin);
    return true;
}

/// 释放所有动态内存 (地图 + 3D 网格)
void HybridAStar::ReleaseMemory() {
    if (map_data_) {
        delete[] map_data_;
        map_data_ = nullptr;
    }

    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            if (state_node_map_[i] == nullptr) continue;
            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr) continue;
                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    delete state_node_map_[i][j][k];
                    state_node_map_[i][j][k] = nullptr;
                }
                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }
            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr;
        }
        delete[] state_node_map_;
        state_node_map_ = nullptr;
    }
    terminal_node_ptr_ = nullptr;
}
