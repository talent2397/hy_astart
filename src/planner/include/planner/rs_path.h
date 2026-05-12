#ifndef HYBRID_ASTAR_PLANNER_RS_PATH_H
#define HYBRID_ASTAR_PLANNER_RS_PATH_H

#include "common/types.h"

#include <cmath>
#include <limits>

/*!
 * @brief Reeds-Shepp 曲线路径规划器
 *
 * RS 曲线是连接任意两个位姿 (x, y, θ) 的最短路径, 允许前进和后退,
 * 由最多 5 段基本运动组成: 左转(L), 右转(R), 直行(S).
 *
 * 参考资料:
 *   1990 "Optimal paths for a car that goes both forwards and backwards."
 *   J. A. Reeds, L. A. Shepp. Pacific Journal of Mathematics.
 *
 * 注意: 论文中有多处公式推导错误, 尤其是 Section 8.
 * 本实现参考了 OMPL (Open Motion Planning Library) 的修正版本.
 *
 * 在 Hybrid A* 中的应用:
 *   当搜索节点距离终点 < shot_distance (如 5m) 时,
 *   尝试用 RS 曲线直连, 如果无碰撞则搜索成功, 大幅加速收敛.
 */
class RSPath {
public:
    RSPath() = delete;

    /**
     * @param turning_radius 最小转弯半径 = wheel_base / tan(max_steering)
     */
    explicit RSPath(double turning_radius = 1.0);

    /// RS 路径中的基本运动类型
    /// L=左转, R=右转, S=直行, N=无操作 (占位)
    enum RSPathSegmentType {
        N = 0, L = 1, S = 2, R = 3
    };

    /// 18 种 RS 路径类型 (来自论文 TABLE 1)
    /// 每种路径由最多 5 段运动组成, 用 N 填充不足 5 段的部分
    /// 示例: {L, R, L, N, N} = 左转 → 右转 → 左转
    static const RSPathSegmentType RS_path_segment_type[18][5];

    /**
     * @brief RS 路径数据 — 存储一条完整的 RS 路径
     *
     * length_[0..4]: 各段的长度 (正值=前进, 负值=后退)
     * type_[0..4]:   各段的运动类型 (L/R/S/N)
     * total_length_: 各段长度绝对值之和
     */
    struct RSPathData {
    public:
        explicit RSPathData(const RSPathSegmentType* type = RS_path_segment_type[0],
                            double t = std::numeric_limits<double>::max(),
                            double u = 0.0, double v = 0.0, double w = 0.0, double x = 0.0) : type_(type) {
            length_[0] = t;
            length_[1] = u;
            length_[2] = v;
            length_[3] = w;
            length_[4] = x;
            total_length_ = std::fabs(length_[0]) + std::fabs(length_[1]) + std::fabs(length_[2])
                            + std::fabs(length_[3]) + std::fabs(length_[4]);
        }

        double Length() const { return total_length_; }

    public:
        double length_[5]{};                // 各段长度 (正向=前进, 负向=后退)
        const RSPathSegmentType* type_;     // 各段类型 (指向 RS_path_segment_type 某一行)

    private:
        double total_length_;
    };

    /**
     * @brief 计算两个位姿之间的实际最短 RS 距离 (用于启发式函数)
     * 输入/输出以 turning_radius 为单位
     */
    double Distance(double x_0, double y_0, double yaw_0,
                    double x_1, double y_1, double yaw_1);

    /**
     * @brief 获取 RS 路径的离散采样点 (用于碰撞检测)
     * @param step_size 采样步长
     * @param length 输出: RS 路径总长度
     * @return 离散路径点序列 (世界坐标)
     */
    TypeVectorVecd<3> GetRSPath(const Vec3d& start_state, const Vec3d& goal_state,
                                double step_size, double& length);

    /// 获取最短 RS 路径的数据结构 (不含采样)
    RSPathData GetRSPath(double x_0, double y_0, double yaw_0,
                         double x_1, double y_1, double yaw_1);

    /// 获取归一化坐标下的 RS 路径 (以 turning_radius 为单位)
    RSPathData GetRSPath(double x, double y, double phi);

private:
    // ---- 五大类 RS 路径组合 (按段数分类) ----
    static void CSC(double x, double y, double phi, RSPathData& path);    // 3 段: 曲线-直线-曲线
    void CCC(double x, double y, double phi, RSPathData& path);           // 3 段: 曲线-曲线-曲线
    static void CCCC(double x, double y, double phi, RSPathData& path);   // 4 段: 曲线*4
    static void CCSC(double x, double y, double phi, RSPathData& path);   // 4 段: 曲线-曲线-直线-曲线
    static void CCSCC(double x, double y, double phi, RSPathData& path);  // 5 段: 曲线-曲线-直线-曲线-曲线

    // ---- 数学工具函数 ----
    static inline double Mod2Pi(double x);
    static inline void Polar(double x, double y, double& r, double& theta);
    static inline void TauOmega(double u, double v, double xi, double eta, double phi, double& tau, double& omega);

    // ---- 论文 Section 8.1 ~ 8.11 的公式实现 ----
    static inline bool LpSpLp(double x, double y, double phi, double& t, double& u, double& v);
    static inline bool LpSpRp(double x, double y, double phi, double& t, double& u, double& v);
    static inline bool LpRmL(double x, double y, double phi, double& t, double& u, double& v);
    static inline bool LpRupLumRm(double x, double y, double phi, double& t, double& u, double& v);
    static inline bool LpRumLumRp(double x, double y, double phi, double& t, double& u, double& v);
    static inline bool LpRmSmLm(double x, double y, double phi, double& t, double& u, double& v);
    static inline bool LpRmSmRm(double x, double y, double phi, double& t, double& u, double& v);
    static inline bool LpRmSLmRp(double x, double y, double phi, double& t, double& u, double& v);

private:
    double turning_radius_ = 1.0;  // 最小转弯半径
};

#endif // HYBRID_ASTAR_PLANNER_RS_PATH_H
