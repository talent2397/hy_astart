#ifndef HYBRID_ASTAR_PLANNER_STATE_NODE_H
#define HYBRID_ASTAR_PLANNER_STATE_NODE_H

#include "common/types.h"

/**
 * @brief Hybrid A* 搜索状态节点 — A* 搜索图中的顶点
 *
 * 每个节点代表车辆在状态空间 (x, y, θ) 中的一个离散状态。
 * 搜索过程中节点经历: NOT_VISITED → IN_OPENSET → IN_CLOSESET
 *
 * 节点使用裸指针管理, 所有节点分配在 state_node_map_ (3D 网格) 中统一管理,
 * openset_ (multimap) 只存指针, 搜索结束后统一释放。
 *
 * Ptr = StateNode* 是裸指针, 因为:
 *   - 节点数量大 (可到数万), shared_ptr 开销太高
 *   - 节点生命周期由 state_node_map_ 统一管理, 不需要引用计数
 */
struct StateNode {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// A* 搜索中节点的三种状态
    enum NODE_STATUS {
        NOT_VISITED = 0,   // 尚未访问
        IN_OPENSET = 1,    // 在 openset (待扩展队列) 中
        IN_CLOSESET = 2    // 已扩展, 在 closeset 中
    };

    /// 从父节点到当前节点的运动方向
    enum DIRECTION {
        FORWARD = 0,       // 前进
        BACKWARD = 1,      // 倒车
        NO = 3             // 无方向 (起点/终点)
    };

    StateNode() = delete;

    /// 构造时仅设置网格索引, 其余字段在搜索过程中填充
    explicit StateNode(const Vec3i& grid_index) {
        node_status_ = NOT_VISITED;
        grid_index_ = grid_index;
        parent_node_ = nullptr;
    }

    /// Reset 用于清除搜索中间状态 (配合 HybridAStar::Reset())
    void Reset() {
        node_status_ = NOT_VISITED;
        parent_node_ = nullptr;
    }

    NODE_STATUS node_status_;          // 当前搜索状态
    DIRECTION direction_{};            // 从父节点来的运动方向

    Vec3d state_;                      // 连续状态 (x, y, θ) — 世界坐标
    Vec3i grid_index_;                 // 离散化后的网格索引 (gx, gy, gφ)

    double g_cost_{};                  // g(n): 从起点到当前节点的实际代价
    double f_cost_{};                  // f(n) = g(n) + h(n): A* 总估价
    int steering_grade_{};             // 转向档位 (-N ~ +N), 0 = 直线前进

    StateNode* parent_node_;           // 父节点指针 (用于回溯路径)
    using Ptr = StateNode*;            // 类型别名: 节点指针

    /// 从父节点到当前节点的中间状态序列 (用于碰撞检测和路径提取)
    VectorVec3d intermediate_states_;
};

#endif // HYBRID_ASTAR_PLANNER_STATE_NODE_H
