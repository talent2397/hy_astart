#ifndef HYBRID_ASTAR_PLANNER_RS_PATH_H
#define HYBRID_ASTAR_PLANNER_RS_PATH_H

#include "common/types.h"

#include <cmath>
#include <limits>

/*!
 * Reeds-Shepp 曲线路径规划器
 *
 * Refer to:
 *   1990 Optimal paths for a car that goes both forwards and backwards.
 *   J. A. Reeds, L. A. Shepp. Pacific J.
 *
 * 注意：论文中有多处公式推导错误，尤其是 Section 8。
 */
class RSPath {
public:
    RSPath() = delete;
    explicit RSPath(double turning_radius = 1.0);

    enum RSPathSegmentType {
        N = 0, L = 1, S = 2, R = 3
    };
    static const RSPathSegmentType RS_path_segment_type[18][5];

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
        double length_[5]{};
        const RSPathSegmentType* type_;

    private:
        double total_length_;
    };

    // 计算 (x0,y0,yaw0) 到 (x1,y1,yaw1) 的实际最短距离
    double Distance(double x_0, double y_0, double yaw_0,
                    double x_1, double y_1, double yaw_1);

    // 获取 RS 路径离散点
    TypeVectorVecd<3> GetRSPath(const Vec3d& start_state, const Vec3d& goal_state,
                                double step_size, double& length);

    RSPathData GetRSPath(double x_0, double y_0, double yaw_0,
                         double x_1, double y_1, double yaw_1);

    RSPathData GetRSPath(double x, double y, double phi);

private:
    static void CSC(double x, double y, double phi, RSPathData& path);
    void CCC(double x, double y, double phi, RSPathData& path);
    static void CCCC(double x, double y, double phi, RSPathData& path);
    static void CCSC(double x, double y, double phi, RSPathData& path);
    static void CCSCC(double x, double y, double phi, RSPathData& path);
    static inline double Mod2Pi(double x);
    static inline void Polar(double x, double y, double& r, double& theta);
    static inline void TauOmega(double u, double v, double xi, double eta, double phi, double& tau, double& omega);

    // 论文 Section 8.1 ~ 8.11
    static inline bool LpSpLp(double x, double y, double phi, double& t, double& u, double& v);
    static inline bool LpSpRp(double x, double y, double phi, double& t, double& u, double& v);
    static inline bool LpRmL(double x, double y, double phi, double& t, double& u, double& v);
    static inline bool LpRupLumRm(double x, double y, double phi, double& t, double& u, double& v);
    static inline bool LpRumLumRp(double x, double y, double phi, double& t, double& u, double& v);
    static inline bool LpRmSmLm(double x, double y, double phi, double& t, double& u, double& v);
    static inline bool LpRmSmRm(double x, double y, double phi, double& t, double& u, double& v);
    static inline bool LpRmSLmRp(double x, double y, double phi, double& t, double& u, double& v);

private:
    double turning_radius_ = 1.0;
};

#endif // HYBRID_ASTAR_PLANNER_RS_PATH_H
