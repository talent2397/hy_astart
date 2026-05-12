#ifndef HYBRID_ASTAR_PLANNER_TRAJECTORY_OPTIMIZER_H
#define HYBRID_ASTAR_PLANNER_TRAJECTORY_OPTIMIZER_H

#include "common/types.h"
#include <cmath>

/**
 * @brief 轨迹后端优化器 (梯度下降平滑)
 *
 * 对前端 Hybrid A* 搜索得到的路径进行平滑优化, 优化目标包含三项:
 *   1. 曲率项 (CurvatureTerm):   惩罚大曲率, 使路径更平滑可驾驶
 *   2. 障碍物项 (ObstacleTerm):   推开离障碍物太近的路径点
 *   3. 平滑项 (SmoothTerm):      减少相邻点间的锯齿
 *
 * 注意: 原作者评价此优化器"效果一般",
 * 前端 Hybrid A* 搜索的轨迹已基本满足需求, 此优化器默认未启用.
 *
 * 模板参数:
 *   CheckCollisionFunction: 碰撞检测函数 bool(double, double, double)
 *   NearestObstacleFunction: 最近障碍物查询 Vec2d(double, double)
 *
 * 参考: "Practical Search Techniques in Path Planning for Autonomous Driving"
 */
class TrajectoryOptimizer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TrajectoryOptimizer() = default;

    /**
     * @brief 对路径进行梯度下降优化
     * @param CheckCollision  碰撞检测回调
     * @param NearestObstacle 最近障碍物查询回调
     * @param path            原始路径 (VectorVec3d: x,y,θ)
     * @return 优化后的路径
     */
    template<typename CheckCollisionFunction, typename NearestObstacleFunction>
    VectorVec3d Optimize(CheckCollisionFunction CheckCollision, NearestObstacleFunction NearestObstacle,
                         const VectorVec3d& path) {
        const unsigned int N = path.size();
        VecXd grad_f;
        grad_f.resize(2 * N);          // 2N: 每个点的 x,y 梯度
        grad_f.setZero();

        // 将路径的 (x,y) 展平为一维向量方便梯度计算
        VecXd path_matrix(N * 2);
        for (unsigned int i = 0; i < N; ++i) {
            path_matrix.block(i * 2, 0, 2, 1) = path[i].head(2);
        }

        VecXd r, prev_d, prev_r;
        unsigned int max_iteration = 100u;  // 固定 100 次迭代

        for (unsigned int i = 0; i < max_iteration; ++i) {
            grad_f.setZero();
            // 对内部点 (跳过首尾各 2 个点) 计算梯度
            for (unsigned int j = 2; j < N - 2; ++j) {
                const Vec2d& x_2_j = path_matrix.block((j - 2) * 2, 0, 2, 1);
                const Vec2d& x_1_j = path_matrix.block((j - 1) * 2, 0, 2, 1);
                const Vec2d& x_j = path_matrix.block(j * 2, 0, 2, 1);
                const Vec2d& x_j_1 = path_matrix.block((j + 1) * 2, 0, 2, 1);
                const Vec2d& x_j_2 = path_matrix.block((j + 2) * 2, 0, 2, 1);

                // 三项梯度的加权和
                grad_f.block(2 * j, 0, 2, 1) += w_k * CurvatureTerm(x_1_j, x_j, x_j_1);
                grad_f.block(2 * j, 0, 2, 1) += w_o * ObstacleTerm(x_j, NearestObstacle(x_j.x(), x_j.y()));
                grad_f.block(2 * j, 0, 2, 1) += w_s * SmoothTerm(x_2_j, x_1_j, x_j, x_j_1, x_j_2);
            }
            prev_d = -grad_f;
            // 梯度下降更新: x = x - α * grad (这里用加负梯度)
            path_matrix = path_matrix + alpha_ * prev_d * (w_k + w_s + w_o);
        }

        // 从优化后的 (x,y) 重建带 θ 的路径点
        VectorVec3d optimized_path;
        for (unsigned int i = 0; i < N - 1; ++i) {
            Vec2d delta = path_matrix.block(2 * (i + 1), 0, 2, 1)
                          - path_matrix.block(2 * i, 0, 2, 1);
            double theta = std::atan2(delta.y(), delta.x());  // 朝向 = 相邻点连线方向
            optimized_path.emplace_back(Vec3d(path_matrix(2 * i, 0),
                                              path_matrix(2 * i + 1, 0), theta));
        }
        return optimized_path;
    }

private:
    /// 值钳位到 [bound1, bound2] 范围
    template<typename T>
    T Clamp(const T value, T bound1, T bound2) {
        if (bound1 > bound2) std::swap(bound1, bound2);
        if (value < bound1) return bound1;
        if (value > bound2) return bound2;
        return value;
    }

    /**
     * @brief 障碍物排斥梯度: 如果路径点离障碍物太近 (< d_max), 施加远离力
     * 梯度方向: 从障碍物指向路径点
     * 梯度大小: 2 * (distance - d_max)
     */
    inline Vec2d ObstacleTerm(const Vec2d& x_i, const Vec2d& obstacle_coord) const {
        Vec2d gradient = Vec2d::Zero();
        double obstacle_distance = (x_i - obstacle_coord).norm();
        if (obstacle_distance <= d_max) {
            gradient = 2 * (obstacle_distance - d_max) * (x_i - obstacle_coord).normalized();
        }
        return gradient;
    }

    /// 五对角平滑项: 最小化二阶差分, 使路径点均匀分布
    static inline Vec2d SmoothTerm(const Vec2d& x_2_j, const Vec2d& x_1_j, const Vec2d& x_j,
                                   const Vec2d& x_j_1, const Vec2d& x_j_2) {
        return 2.0 * (x_2_j - 4.0 * x_1_j + 6 * x_j - 4 * x_j_1 + x_j_2);
    }

    /**
     * @brief 曲率约束梯度: 惩罚超过 k_max 的局部曲率
     * 用三点 (x_{j-1}, x_j, x_{j+1}) 计算局部曲率
     */
    inline Vec2d CurvatureTerm(const Vec2d& x_1_j, const Vec2d& x_j, const Vec2d& x_j_1) {
        Vec2d delta_x_j = x_j - x_1_j;
        Vec2d delta_x_j_1 = x_j_1 - x_j;
        double delta_x_j_norm = delta_x_j.norm();
        double delta_x_j_1_norm = delta_x_j_1.norm();

        if (delta_x_j_norm > 0.0 && delta_x_j_1_norm > 0.0) {
            double delta_phi = DeltaPhi(x_1_j, x_j, x_j_1);
            if (delta_phi / delta_x_j_norm <= k_max) return Vec2d::Zero();

            double temp_1 = 1.0 / delta_x_j_norm;
            double temp_2 = -1.0 / std::sqrt(1 - std::pow(std::cos(delta_phi), 2));
            double temp_4 = delta_phi / delta_x_j_norm / delta_x_j_norm;

            Vec2d p1 = OrthogonalComplement(delta_x_j, -delta_x_j_1) / delta_x_j_1.norm() / delta_x_j.norm();
            Vec2d p2 = OrthogonalComplement(-delta_x_j_1, delta_x_j) / delta_x_j_1.norm() / delta_x_j.norm();
            Vec2d temp_3 = -p1 - p2;

            Vec2d J_1_j = 2.0 * (delta_phi / delta_x_j_norm - k_max) *
                          (temp_1 * temp_2 * p2 + temp_4 * delta_x_j.normalized());
            Vec2d J_j = 2.0 * (delta_phi / delta_x_j_norm - k_max) *
                        (temp_1 * temp_2 * temp_3 - temp_4 * delta_x_j.normalized());
            Vec2d J_j_1 = 2.0 * (delta_phi / delta_x_j_norm - k_max) * (temp_1 * temp_2 * p1);

            Vec2d grad = 0.25 * J_1_j + 0.5 * J_j + 0.25 * J_j_1;
            if (std::isnan(grad.x()) || std::isnan(grad.y())) return Vec2d::Zero();
            return grad;
        }
        return Vec2d::Zero();
    }

    /// 计算三点转角: x0→x1 与 x1→x2 之间的夹角
    inline double DeltaPhi(const Vec2d& x0, const Vec2d& x1, const Vec2d& x2) {
        Vec2d delta_x0 = x1 - x0;
        Vec2d delta_x1 = x2 - x1;
        double project_length = delta_x0.transpose() * delta_x1;
        return std::acos(Clamp<double>(project_length / (delta_x0.norm() * delta_x1.norm()), -1, 1));
    }

    /// 正交补: 从 a 中去除 b 方向的分量, 得到 a ⊥ b
    static inline Vec2d OrthogonalComplement(const Vec2d& a, const Vec2d& b) {
        return a - a.transpose() * b.normalized() * b.normalized();
    }

private:
    double alpha_ = 0.1;    // 梯度下降步长
    double w_o = 0.05;      // 障碍物项权重
    double w_k = 0.01;      // 曲率项权重
    double w_s = 0.2;       // 平滑项权重
    double k_max = 0.01;    // 最大允许曲率
    double d_max = 3.0;     // 障碍物影响距离 (米)
};

#endif // HYBRID_ASTAR_PLANNER_TRAJECTORY_OPTIMIZER_H
