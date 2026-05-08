#ifndef HYBRID_ASTAR_PLANNER_STATE_NODE_H
#define HYBRID_ASTAR_PLANNER_STATE_NODE_H

#include "common/types.h"

/**
 * @brief Hybrid A* 搜索状态节点
 */
struct StateNode {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum NODE_STATUS {
        NOT_VISITED = 0, IN_OPENSET = 1, IN_CLOSESET = 2
    };

    enum DIRECTION {
        FORWARD = 0, BACKWARD = 1, NO = 3
    };

    StateNode() = delete;

    explicit StateNode(const Vec3i& grid_index) {
        node_status_ = NOT_VISITED;
        grid_index_ = grid_index;
        parent_node_ = nullptr;
    }

    void Reset() {
        node_status_ = NOT_VISITED;
        parent_node_ = nullptr;
    }

    NODE_STATUS node_status_;
    DIRECTION direction_{};

    Vec3d state_;
    Vec3i grid_index_;

    double g_cost_{}, f_cost_{};
    int steering_grade_{};

    StateNode* parent_node_;
    using Ptr = StateNode*;

    VectorVec3d intermediate_states_;
};

#endif // HYBRID_ASTAR_PLANNER_STATE_NODE_H
