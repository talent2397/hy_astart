/*******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022 Zhang Zhimeng
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/**
 * @file rs_path.cpp
 * @brief Reeds-Shepp 曲线实现 — 连接两个任意位姿的最短路径
 *
 * RS 曲线由最多 5 段基本运动组成:
 *   L = 左转 (Left turn, 沿半径为 r 的圆弧)
 *   R = 右转 (Right turn)
 *   S = 直行 (Straight)
 *   N = 空操作 (占位, 不足 5 段时填充)
 *
 * 论文定义了 18 种路径类型 (TABLE 1), 本文件实现全部 18 种。
 *
 * 工作流程:
 *   1. 将起点→终点的相对位姿变换到以起点为原点的局部坐标系
 *   2. 除以 turning_radius 做归一化 (此时最小转弯半径 = 1)
 *   3. 依次尝试所有 18 种路径类型, 取总长最短的
 *   4. 对最短路径做等距采样, 变换回世界坐标
 *
 * 参考:
 *   "Optimal paths for a car that goes both forwards and backwards."
 *   J. A. Reeds, L. A. Shepp. Pacific J. Math. 1990
 *
 * 注意: 论文 Section 8 有多处公式推导错误,
 * 本实现参考了 OMPL 的修正版本.
 */

#include "planner/rs_path.h"

#include <glog/logging.h>

// ============================================================
// P 371: TABLE 1 — 18 种 RS 路径类型
//
// 每种路径由最多 5 段基本运动组成, N 表示该段不存在
// 示例: {L, R, L, N, N} = 左转 → 右转 → 左转 (3 段)
//       {L, R, S, L, R} = 左转 → 右转 → 直行 → 左转 → 右转 (5 段)
// ============================================================
const RSPath::RSPathSegmentType RSPath::RS_path_segment_type[18][5] = {
        {L, R, L, N, N},     // 0:  CCC   类
        {R, L, R, N, N},     // 1:  CCC   类
        {L, R, L, R, N},     // 2:  CCCC  类
        {R, L, R, L, N},     // 3:  CCCC  类
        {L, R, S, L, N},     // 4:  CCSC  类
        {R, L, S, R, N},     // 5:  CCSC  类
        {L, S, R, L, N},     // 6:  CCSC  类
        {R, S, L, R, N},     // 7:  CCSC  类
        {L, R, S, R, N},     // 8:  CCSC  类
        {R, L, S, L, N},     // 9:  CCSC  类
        {R, S, R, L, N},     // 10: CCSC  类
        {L, S, L, R, N},     // 11: CCSC  类
        {L, S, R, N, N},     // 12: CSC   类
        {R, S, L, N, N},     // 13: CSC   类
        {L, S, L, N, N},     // 14: CSC   类
        {R, S, R, N, N},     // 15: CSC   类
        {L, R, S, L, R},     // 16: CCSCC 类
        {R, L, S, R, L}      // 17: CCSCC 类
};

RSPath::RSPath(double turning_radius) : turning_radius_(turning_radius) {}

// ============================================================
// 数学工具函数
// ============================================================

/// 角度归一化到 (-π, π]
double RSPath::Mod2Pi(double x) {
    double v = fmod(x, 2 * M_PI);

    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

/// 直角坐标 → 极坐标
void RSPath::Polar(double x, double y, double &r, double &theta) {
    r = std::sqrt(x * x + y * y);
    theta = std::atan2(y, x);
}

/**
 * @brief 计算 τ 和 ω (论文公式 8.3)
 *
 * τ 是使两段圆弧平滑连接的角度参数
 * ω 是由 τ 导出的相位差
 *
 * 公式: tau = Mod2Pi(t1) 或 t1+π, 取决于 t2 的符号
 *       omega = Mod2Pi(tau - u + v - phi)
 */
void RSPath::TauOmega(double u, double v, double xi, double eta, double phi, double &tau, double &omega) {
    double delta = Mod2Pi(u - v);
    double A = std::sin(u) - std::sin(delta);
    double B = std::cos(u) - std::cos(delta) - 1.0;
    double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
    double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
    tau = (t2 < 0.0) ? Mod2Pi(t1 + M_PI) : Mod2Pi(t1);
    omega = Mod2Pi(tau - u + v - phi);
}

// ============================================================
// Section 8.1 ~ 8.11: 论文公式实现
//
// 命名规则:
//   L/R/S = 运动类型 (Left/Right/Straight)
//   p/m   = 前进/后退 (plus/minus)
//
//   例如 LpSpRp = 前进左转 → 前进直行 → 前进右转
//        LpRmL  = 前进左转 → 后退右转 → 前进左转
//
// 每个函数:
//   输入 (x, y, phi): 终点相对于起点的归一化位姿
//   输出 (t, u, v): 各段长度
//   返回 bool: 这种路径类型的公式是否有有效解
// ============================================================

// ---- 8.1: L+S+L+ (前进左转 → 前进直行 → 前进左转) ----
bool RSPath::LpSpLp(double x, double y, double phi, double &t, double &u, double &v) {
    Polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);

    if (t >= 0.0) {
        v = Mod2Pi(phi - t);
        if (v >= 0.0) {
            return true;
        }
    }

    return false;
}

// ---- 8.2: L+S+R+ (前进左转 → 前进直行 → 前进右转) ----
bool RSPath::LpSpRp(double x, double y, double phi, double &t, double &u, double &v) {
    double t1, u1;
    Polar(x + std::sin(phi), y - 1 - std::cos(phi), u1, t1);
    u1 = std::pow(u1, 2);

    if (u1 < 4.0) {
        return false;  // 无解
    }

    double theta;
    u = std::sqrt(u1 - 4.0);
    theta = std::atan2(2.0, u);
    t = Mod2Pi(t1 + theta);
    v = Mod2Pi(t - phi);

    return true;
}

// ---- 外部接口: 获取两个位姿之间的最短 RS 路径 ----

/**
 * @brief 计算两个世界坐标位姿之间的最短 RS 路径 (归一化后调用内部方法)
 *
 * 步骤:
 *   1. 计算相对位移 (dx, dy)
 *   2. 旋转到起点坐标系: x = cos(θ₀)*dx + sin(θ₀)*dy, y = -sin(θ₀)*dx + cos(θ₀)*dy
 *   3. 相对角度 phi = θ₁ - θ₀
 *   4. 除以 turning_radius 做归一化
 */
RSPath::RSPathData RSPath::GetRSPath(const double x_0, const double y_0, const double yaw_0,
                                     const double x_1, const double y_1, const double yaw_1) {
    // 平移: 计算相对位移
    double dx = x_1 - x_0;
    double dy = y_1 - y_0;

    // 旋转到起点坐标系: 2D 旋转矩阵的逆
    double c = std::cos(yaw_0);  // cos(θ₀)
    double s = std::sin(yaw_0);  // sin(θ₀)
    double x = c * dx + s * dy;  // 局部坐标 X
    double y = -s * dx + c * dy;  // 局部坐标 Y
    double phi = yaw_1 - yaw_0;   // 相对角度

    // 归一化 (以 turning_radius 为单位)
    return GetRSPath(x / turning_radius_, y / turning_radius_, phi);
}

/**
 * @brief 归一化坐标下的 RS 路径求解
 *
 * 依次尝试 5 大类路径组合, 取总长最短的:
 *   CSC   (3段): 曲线-直线-曲线
 *   CCC   (3段): 曲线-曲线-曲线
 *   CCCC  (4段): 曲线×4
 *   CCSC  (4段): 曲线-曲线-直线-曲线
 *   CCSCC (5段): 曲线-曲线-直线-曲线-曲线
 *
 * 每个大类尝试最多 8 种镜像组合 (x→-x, y→-y, phi→-phi),
 * 总共最多 48 种子情况, 取最短的。
 */
RSPath::RSPathData RSPath::GetRSPath(const double x, const double y, const double phi) {
    RSPathData path;
    CSC(x, y, phi, path);
    CCC(x, y, phi, path);
    CCCC(x, y, phi, path);
    CCSC(x, y, phi, path);
    CCSCC(x, y, phi, path);

    return path;
}

/// 计算两个世界坐标位姿之间的最短 RS 距离
double RSPath::Distance(const double x_0, const double y_0, const double yaw_0,
                        const double x_1, const double y_1, const double yaw_1) {
    return turning_radius_ * GetRSPath(x_0, y_0, yaw_0, x_1, y_1, yaw_1).Length();
}

// ============================================================
// CSC (曲线-直线-曲线) 大类: 3 段, 8 种镜像组合
//
// 覆盖路径类型: 12, 13, 14, 15
// 子公式: LpSpLp (8.1), LpSpRp (8.2)
// 每种子公式尝试 4 种镜像 (x/-x, y/-y 组合)
// ============================================================
void RSPath::CSC(double x, double y, double phi, RSPathData &path) {
    double t, u, v, length_min = path.Length(), L;

    // L+S+L+ 的 4 种镜像
    if (LpSpLp(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[14], t, u, v);
        length_min = L;
    }

    if (LpSpLp(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[14], -t, -u, -v);
        length_min = L;
    }

    if (LpSpLp(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[15], t, u, v);
        length_min = L;
    }

    if (LpSpLp(-x, -y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[15], -t, -u, -v);
        length_min = L;
    }

    // L+S+R+ 的 4 种镜像
    if (LpSpRp(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[12], t, u, v);
        length_min = L;
    }

    if (LpSpRp(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[12], -t, -u, -v);
        length_min = L;
    }

    if (LpSpRp(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[13], t, u, v);
        length_min = L;
    }

    if (LpSpRp(-x, -y, phi, t, u, v) && length_min > std::fabs(t) + std::fabs(u) + std::fabs(v)) {
        path = RSPathData(RS_path_segment_type[13], -t, -u, -v);
    }
}

// ---- 8.3: L+R-L+ (前进左转 → 后退右转 → 前进左转) ----
bool RSPath::LpRmL(double x, double y, double phi, double &t, double &u, double &v) {
    double xi = x - std::sin(phi);
    double eta = y - 1.0 + std::cos(phi);
    double u1, theta;
    Polar(xi, eta, u1, theta);

    if (u1 > 4.0) {
        return false;
    }

    u = -2.0 * std::asin(0.25 * u1);  // u < 0 表示后退
    t = Mod2Pi(theta + 0.5 * u + M_PI);
    v = Mod2Pi(phi - t + u);

    return true;
}

// ============================================================
// CCC (曲线-曲线-曲线) 大类: 3 段, 12 种镜像组合
//
// 覆盖路径类型: 0, 1
// 子公式: LpRmL (8.3)
// 除了 x/phi 正负镜像, 还包含坐标变换 (绕终点旋转) 的情况
// ============================================================
void RSPath::CCC(double x, double y, double phi, RSPathData &path) {
    double t, u, v, L;
    double length_min = path.Length();

    // 4 种标准镜像
    if (LpRmL(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[0], t, u, v);
        length_min = L;
    }

    if (LpRmL(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[0], -t, -u, -v);
        length_min = L;
    }

    if (LpRmL(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[1], t, u, v);
        length_min = L;
    }

    if (LpRmL(-x, -y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[1], -t, -u, -v);
        length_min = L;
    }

    // 4 种绕终点坐标系旋转的情况:
    // xb = x*cos(phi) + y*sin(phi), yb = x*sin(phi) - y*cos(phi)
    // 此时参数顺序变为 (v, u, t) 而非 (t, u, v)
    double xb = x * std::cos(phi) + y * std::sin(phi);
    double yb = x * std::sin(phi) - y * std::cos(phi);
    if (LpRmL(xb, yb, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[0], v, u, t);
        length_min = L;
    }

    if (LpRmL(-xb, yb, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[0], -v, -u, -t);
        length_min = L;
    }

    if (LpRmL(xb, -yb, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[1], v, u, t);
        length_min = L;
    }

    if (LpRmL(-xb, -yb, phi, t, u, v) && length_min > std::fabs(t) + std::fabs(u) + std::fabs(v)) {
        path = RSPathData(RS_path_segment_type[1], -v, -u, -t);
    }
}

// ---- 8.4: L+R-L-R- (前进左转 → 前进右转 → 后退左转 → 后退右转) ----
bool RSPath::LpRupLumRm(double x, double y, double phi, double &t, double &u, double &v) {
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));

    if (rho > 1.0) {
        return false;  // 超出可达范围
    }

    u = std::acos(rho);  // u > 0 表示前进
    TauOmega(u, -u, xi, eta, phi, t, v);

    return true;
}

// ---- 8.4 (续): L+R-L-R+ (前进左转 → 后退右转 → 后退左转 → 前进右转) ----
bool RSPath::LpRumLumRp(double x, double y, double phi, double &t, double &u, double &v) {
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    double rho = (20.0 - xi * xi - eta * eta) / 16.0;

    if (rho >= 0.0 && rho <= 1.0) {
        u = -std::acos(rho);  // u < 0 表示后退
        if (u >= -M_PI_2) {
            TauOmega(u, u, xi, eta, phi, t, v);
            return true;
        }
    }

    return false;
}

// ============================================================
// CCCC (曲线×4) 大类: 4 段, 8 种镜像组合
//
// 覆盖路径类型: 2, 3
// 子公式: LpRupLumRm (8.4a), LpRumLumRp (8.4b)
// ============================================================
void RSPath::CCCC(double x, double y, double phi, RSPathData &path) {
    double t, u, v, L;
    double length_min = path.Length();

    // LpRupLumRm 的 4 种镜像
    if (LpRupLumRm(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[2], t, u, -u, v);
        length_min = L;
    }

    if (LpRupLumRm(-x, y, -phi, t, u, v) &&
        length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[2], -t, -u, u, -v);
        length_min = L;
    }

    if (LpRupLumRm(x, -y, -phi, t, u, v) &&
        length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[3], t, u, -u, v);
        length_min = L;
    }

    if (LpRupLumRm(-x, -y, phi, t, u, v) &&
        length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[3], -t, -u, u, -v);
        length_min = L;
    }

    // LpRumLumRp 的 4 种镜像
    if (LpRumLumRp(x, y, phi, t, u, v) &&
        length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[2], t, u, u, v);
        length_min = L;
    }

    if (LpRumLumRp(-x, y, -phi, t, u, v) &&
        length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[2], -t, -u, -u, -v);
        length_min = L;
    }

    if (LpRumLumRp(x, -y, -phi, t, u, v) &&
        length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[3], t, u, u, v);
        length_min = L;
    }

    if (LpRumLumRp(-x, -y, phi, t, u, v) && length_min > std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v)) {
        path = RSPathData(RS_path_segment_type[3], -t, -u, -u, -v);
    }
}

// ---- 8.7: L+R-S-L- (前进左转 → 后退右转 → 前进直行 → 后退左转) ----
bool RSPath::LpRmSmLm(double x, double y, double phi, double &t, double &u, double &v) {
    double xi = x - std::sin(phi);
    double eta = y - 1.0 + std::cos(phi);
    double rho, theta;

    Polar(xi, eta, rho, theta);

    if (rho < 2.0) {
        return false;
    }

    double r = std::sqrt(rho * rho - 4.0);
    u = 2.0 - r;
    t = Mod2Pi(theta + std::atan2(r, -2.0));
    v = Mod2Pi(phi - M_PI_2 - t);

    return true;
}

// ---- 8.8: L+R-S-R- (前进左转 → 后退右转 → 前进直行 → 后退右转) ----
bool RSPath::LpRmSmRm(double x, double y, double phi, double &t, double &u, double &v) {
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    double rho, theta;

    Polar(-eta, xi, rho, theta);

    if (rho < 2.0) {
        return false;
    }

    t = theta;
    u = 2.0 - rho;
    v = Mod2Pi(t + M_PI_2 - phi);

    return true;
}

// ============================================================
// CCSC (曲线-曲线-直线-曲线) 大类: 4 段, 16 种镜像组合
//
// 覆盖路径类型: 4, 5, 6, 7, 8, 9, 10, 11
// 子公式: LpRmSmLm (8.7), LpRmSmRm (8.8)
// ============================================================
void RSPath::CCSC(double x, double y, double phi, RSPathData &path) {
    double t, u, v, L;
    double length_min = path.Length() - M_PI_2;

    // LpRmSmLm 的 4 种镜像
    if (LpRmSmLm(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[4], t, -M_PI_2, u, v);
        length_min = L;
    }

    if (LpRmSmLm(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[4], -t, M_PI_2, -u, -v);
        length_min = L;
    }

    if (LpRmSmLm(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[5], t, -M_PI_2, u, v);
        length_min = L;
    }

    if (LpRmSmLm(-x, -y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[5], -t, M_PI_2, -u, -v);
        length_min = L;
    }

    // LpRmSmRm 的 4 种镜像
    if (LpRmSmRm(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[8], t, -M_PI_2, u, v);
        length_min = L;
    }

    if (LpRmSmRm(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[8], -t, M_PI_2, -u, -v);
        length_min = L;
    }

    if (LpRmSmRm(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[9], t, -M_PI_2, u, v);
        length_min = L;
    }

    if (LpRmSmRm(-x, -y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[9], -t, M_PI_2, -u, -v);
        length_min = L;
    }

    // 绕终点坐标系旋转的情况 (xb, yb)
    double xb = x * std::cos(phi) + y * std::sin(phi);
    double yb = x * std::sin(phi) - y * std::cos(phi);

    // LpRmSmLm 旋转后的 4 种镜像
    if (LpRmSmLm(xb, yb, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[6], v, u, -M_PI_2, t);
        length_min = L;
    }

    if (LpRmSmLm(-xb, yb, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[6], -v, -u, M_PI_2, -t);
        length_min = L;
    }

    if (LpRmSmLm(xb, -yb, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[7], v, u, -M_PI_2, t);
        length_min = L;
    }

    if (LpRmSmLm(-xb, -yb, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[7], -v, -u, M_PI_2, -t);
        length_min = L;
    }

    // LpRmSmRm 旋转后的 4 种镜像
    if (LpRmSmRm(xb, yb, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[10], v, u, -M_PI_2, t);
        length_min = L;
    }

    if (LpRmSmRm(-xb, yb, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[10], -v, -u, M_PI_2, -t);
        length_min = L;
    }

    if (LpRmSmRm(xb, -yb, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[11], v, u, -M_PI_2, t);
        length_min = L;
    }

    if (LpRmSmRm(-xb, -yb, phi, t, u, v) && length_min > std::fabs(t) + std::fabs(u) + std::fabs(v)) {
        path = RSPathData(RS_path_segment_type[11], -v, -u, M_PI_2, -t);
    }
}

// ---- 8.9: L+R-S-L-R+ (前进左转 → 后退右转 → 前进直行 → 后退左转 → 前进右转) ----
bool RSPath::LpRmSLmRp(double x, double y, double phi, double &t, double &u, double &v) {
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    double rho, theta;

    Polar(xi, eta, rho, theta);

    if (rho >= 2.0) {
        u = 4.0 - std::sqrt(rho * rho - 4.0);

        if (u <= 0.0) {
            t = Mod2Pi(std::atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
            v = Mod2Pi(t - phi);

            return true;
        }
    }

    return false;
}

// ============================================================
// CCSCC (曲线-曲线-直线-曲线-曲线) 大类: 5 段, 4 种镜像组合
//
// 覆盖路径类型: 16, 17
// 子公式: LpRmSLmRp (8.9)
// ============================================================
void RSPath::CCSCC(double x, double y, double phi, RSPathData &path) {
    double t, u, v, L;
    double length_min = path.Length() - M_PI;

    if (LpRmSLmRp(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[16], t, -M_PI_2, u, -M_PI_2, v);
        length_min = L;
    }

    if (LpRmSLmRp(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[16], -t, M_PI_2, -u, M_PI_2, -v);
        length_min = L;
    }

    if (LpRmSLmRp(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[17], t, -M_PI_2, u, -M_PI_2, v);
        length_min = L;
    }

    if (LpRmSLmRp(-x, -y, phi, t, u, v) && length_min > std::fabs(t) + std::fabs(u) + std::fabs(v)) {
        path = RSPathData(RS_path_segment_type[17], -t, M_PI_2, -u, M_PI_2, -v);
    }
}

// ============================================================
// 路径离散化: 将 RS 路径参数转换为世界坐标系中的采样点序列
//
// 对每段运动 (L/R/S) 沿长度参数 t ∈ [0, total_length] 做等距采样:
//   L (左转): x += sin(phi+v) - sin(phi),  y += -cos(phi+v) + cos(phi), phi += v
//   R (右转): x += -sin(phi-v) + sin(phi), y += cos(phi-v) - cos(phi),   phi -= v
//   S (直行): x += v*cos(phi),             y += v*sin(phi),              phi 不变
// ============================================================
TypeVectorVecd<3> RSPath::GetRSPath(const Vec3d &start_state, const Vec3d &goal_state,
                                    const double step_size, double &length) {
    // 1. 获取最短 RS 路径参数
    RSPathData rs_path = GetRSPath(start_state.x(), start_state.y(), start_state.z(),
                                   goal_state.x(), goal_state.y(), goal_state.z());

    length = rs_path.Length() * turning_radius_;

    const double path_length = rs_path.Length() * turning_radius_;
    const auto interpolation_number = static_cast<unsigned int> (path_length / step_size);

    double phi;

    TypeVectorVecd<3> path_poses;

    // 2. 沿路径总长相较于参数 t ∈ [0, 1] 做等距采样
    for (unsigned int i = 0; i <= interpolation_number; ++i) {
        double v;
        double t = i * 1.0 / interpolation_number;  // 归一化参数 [0, 1]
        double seg = t * rs_path.Length();           // 当前累计长度 (归一化单位)

        Vec3d temp_pose(0.0, 0.0, start_state.z());  // 起点朝向
        // 3. 按顺序拼接 5 段运动
        for (unsigned int j = 0; j < 5u && seg > 0; ++j) {
            // 取该段在当前参数 t 下的剩余长度
            if (rs_path.length_[j] < 0.0) {
                v = std::max(-seg, rs_path.length_[j]);  // 后退段: v < 0
                seg += v;  // seg 减少 (后退段长度 < 0)
            } else {
                v = std::min(seg, rs_path.length_[j]);    // 前进段: v > 0
                seg -= v;  // seg 减少 (前进段长度 > 0)
            }

            // 4. 根据运动类型 (L/R/S/N) 更新位姿
            phi = temp_pose.z();
            switch (rs_path.type_[j]) {
                case L:  // 左转圆弧
                    temp_pose.x() = std::sin(phi + v) - std::sin(phi) + temp_pose.x();
                    temp_pose.y() = -std::cos(phi + v) + std::cos(phi) + temp_pose.y();
                    temp_pose.z() = phi + v;
                    break;
                case R:  // 右转圆弧
                    temp_pose.x() = -std::sin(phi - v) + std::sin(phi) + temp_pose.x();
                    temp_pose.y() = std::cos(phi - v) - std::cos(phi) + temp_pose.y();
                    temp_pose.z() = phi - v;
                    break;
                case S:  // 直行
                    temp_pose.x() = v * std::cos(phi) + temp_pose.x();
                    temp_pose.y() = v * std::sin(phi) + temp_pose.y();
                    temp_pose.z() = phi;
                    break;
                case N:  // 无操作
                    break;
            }
        }

        // 5. 从归一化坐标变换回世界坐标
        Vec3d pose;
        pose.block<2, 1>(0, 0) = temp_pose.block<2, 1>(0, 0) * turning_radius_
                                 + start_state.block<2, 1>(0, 0);  // 缩放 + 平移
        pose.z() = temp_pose.z();

        path_poses.emplace_back(pose);
    }

    return path_poses;
}
