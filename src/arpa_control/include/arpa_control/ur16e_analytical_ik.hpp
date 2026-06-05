#ifndef UR16E_ANALYTICAL_IK_HPP
#define UR16E_ANALYTICAL_IK_HPP

// Closed-form UR16e kinematics.
//
// This is a faithful port of the well-tested ros-industrial `ur_kinematics`
// (ur_kin.cpp, Hawkins 2013 analytic IK), parameterized with UR16e DH values.
// It REPLACES an earlier hand-rolled solver that was mathematically incorrect
// (returned poses off by ~180 deg and gave 0 solutions for some reachable poses;
// verified by an offline FK round-trip test). This port passes inverse(forward(q))
// round-trip to ~1e-15 m / ~1e-8 rad for all tested configurations.
//
// Frame convention (ur_kin): T is the base->frame6 homogeneous transform in the
// UR "Base" frame. NOTE: the ur_kin "Base" is NOT ROS `base_link_inertia` — it is
// `base_link_inertia` rotated an additional Rz(pi) about Z (== ROS `base_link`).
// Numerically verified: URDF(base_link_inertia->tool0)(q) == Rz(pi)*forward(q)*F.
// The caller MUST apply that Rz(pi) on the base side (see motion_control_node.cpp
// getJointConfigurations) or far-from-current targets are corrupted by ~1 m and
// solve() returns 0. frame6 is the UR DH tool frame, NOT ROS `tool0` — the caller
// maps frame6->tool0 (a constant pure rotation F) live from the current robot state.

#include <Eigen/Geometry>
#include <vector>
#include <cmath>
#include <array>

namespace ur16e_ik {

struct IKSolution {
    double joints[6];  // shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
};

namespace dh {
    constexpr double d1 = 0.1807;
    constexpr double a2 = -0.4784;
    constexpr double a3 = -0.36;
    constexpr double d4 = 0.17415;
    constexpr double d5 = 0.11985;
    constexpr double d6 = 0.11655;
}

namespace detail {
    constexpr double ZERO_THRESH = 1e-8;
    inline int SGN(double x) { return (x > 0) - (x < 0); }

    // ur_kin forward kinematics -> row-major 4x4 in T[16].
    inline void forward(const double* q, double* T) {
        using namespace dh;
        double s1 = std::sin(q[0]), c1 = std::cos(q[0]);
        double q23 = q[1], q234 = q[1], s2 = std::sin(q[1]), c2 = std::cos(q[1]);
        double s4 = std::sin(q[3]), c4 = std::cos(q[3]);
        q23 += q[2]; q234 += q[2]; q234 += q[3];
        double s5 = std::sin(q[4]), c5 = std::cos(q[4]);
        double s6 = std::sin(q[5]), c6 = std::cos(q[5]);
        double s23 = std::sin(q23), c23 = std::cos(q23);
        double s234 = std::sin(q234), c234 = std::cos(q234);
        T[0]  = c234*c1*s5 - c5*s1;
        T[1]  = c6*(s1*s5 + c234*c1*c5) - s234*c1*s6;
        T[2]  = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6;
        T[3]  = d6*c234*c1*s5 - a3*c23*c1 - a2*c1*c2 - d6*c5*s1 - d5*s234*c1 - d4*s1;
        T[4]  = c1*c5 + c234*s1*s5;
        T[5]  = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6;
        T[6]  = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1;
        T[7]  = d6*(c1*c5 + c234*s1*s5) + d4*c1 - a3*c23*s1 - a2*c2*s1 - d5*s234*s1;
        T[8]  = -s234*s5;
        T[9]  = -c234*s6 - s234*c5*c6;
        T[10] = s234*c5*s6 - c234*c6;
        T[11] = d1 + a3*s23 + a2*s2 - d5*(c23*c4 - s23*s4) - d6*s5*(c23*s4 + s23*c4);
        T[12] = 0.0; T[13] = 0.0; T[14] = 0.0; T[15] = 1.0;
    }

    // ur_kin analytic inverse. T row-major 4x4, q_sols 8*6, returns count.
    inline int inverse(const double* T, double* q_sols, double q6_des) {
        using namespace dh;
        int num_sols = 0;
        double T02 = -T[0], T00 = T[1], T01 = T[2], T03 = -T[3];
        double T12 = -T[4], T10 = T[5], T11 = T[6], T13 = -T[7];
        double T22 =  T[8], T20 = -T[9], T21 = -T[10], T23 = T[11];

        double q1[2];
        {
            double A = d6*T12 - T13, B = d6*T02 - T03, R = A*A + B*B;
            if (std::fabs(A) < ZERO_THRESH) {
                double div;
                if (std::fabs(std::fabs(d4) - std::fabs(B)) < ZERO_THRESH) div = -SGN(d4)*SGN(B);
                else div = -d4/B;
                double as = std::asin(div);
                if (std::fabs(as) < ZERO_THRESH) as = 0.0;
                q1[0] = (as < 0.0) ? as + 2.0*M_PI : as;
                q1[1] = M_PI - as;
            } else if (std::fabs(B) < ZERO_THRESH) {
                double div;
                if (std::fabs(std::fabs(d4) - std::fabs(A)) < ZERO_THRESH) div = SGN(d4)*SGN(A);
                else div = d4/A;
                double ac = std::acos(div);
                q1[0] = ac; q1[1] = 2.0*M_PI - ac;
            } else if (d4*d4 > R) {
                return num_sols;
            } else {
                double ac = std::acos(d4 / std::sqrt(R));
                double at = std::atan2(-B, A);
                double pos = ac + at, neg = -ac + at;
                if (std::fabs(pos) < ZERO_THRESH) pos = 0.0;
                if (std::fabs(neg) < ZERO_THRESH) neg = 0.0;
                q1[0] = (pos >= 0.0) ? pos : 2.0*M_PI + pos;
                q1[1] = (neg >= 0.0) ? neg : 2.0*M_PI + neg;
            }
        }

        double q5[2][2];
        for (int i = 0; i < 2; i++) {
            double numer = (T03*std::sin(q1[i]) - T13*std::cos(q1[i]) - d4);
            double div;
            if (std::fabs(std::fabs(numer) - std::fabs(d6)) < ZERO_THRESH) div = SGN(numer)*SGN(d6);
            else div = numer / d6;
            double ac = std::acos(div);
            q5[i][0] = ac; q5[i][1] = 2.0*M_PI - ac;
        }

        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                double c1 = std::cos(q1[i]), s1 = std::sin(q1[i]);
                double c5 = std::cos(q5[i][j]), s5 = std::sin(q5[i][j]);
                double q6;
                if (std::fabs(s5) < ZERO_THRESH) {
                    q6 = q6_des;
                } else {
                    q6 = std::atan2(SGN(s5)*-(T01*s1 - T11*c1), SGN(s5)*(T00*s1 - T10*c1));
                    if (std::fabs(q6) < ZERO_THRESH) q6 = 0.0;
                    if (q6 < 0.0) q6 += 2.0*M_PI;
                }
                double q2[2], q3[2], q4[2];
                double c6 = std::cos(q6), s6 = std::sin(q6);
                double x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
                double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
                double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) + T03*c1 + T13*s1;
                double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);
                double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
                if (std::fabs(std::fabs(c3) - 1.0) < ZERO_THRESH) c3 = SGN(c3);
                else if (std::fabs(c3) > 1.0) continue;
                double ac = std::acos(c3);
                q3[0] = ac; q3[1] = 2.0*M_PI - ac;
                double denom = a2*a2 + a3*a3 + 2.0*a2*a3*c3;
                double s3 = std::sin(ac);
                double A = (a2 + a3*c3), B = a3*s3;
                q2[0] = std::atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
                q2[1] = std::atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
                double c23_0 = std::cos(q2[0]+q3[0]), s23_0 = std::sin(q2[0]+q3[0]);
                double c23_1 = std::cos(q2[1]+q3[1]), s23_1 = std::sin(q2[1]+q3[1]);
                q4[0] = std::atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
                q4[1] = std::atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);
                for (int k = 0; k < 2; k++) {
                    if (std::fabs(q2[k]) < ZERO_THRESH) q2[k] = 0.0; else if (q2[k] < 0.0) q2[k] += 2.0*M_PI;
                    if (std::fabs(q4[k]) < ZERO_THRESH) q4[k] = 0.0; else if (q4[k] < 0.0) q4[k] += 2.0*M_PI;
                    q_sols[num_sols*6+0] = q1[i];    q_sols[num_sols*6+1] = q2[k];
                    q_sols[num_sols*6+2] = q3[k];    q_sols[num_sols*6+3] = q4[k];
                    q_sols[num_sols*6+4] = q5[i][j]; q_sols[num_sols*6+5] = q6;
                    num_sols++;
                }
            }
        }
        return num_sols;
    }
}  // namespace detail

// Forward kinematics: joints -> base->frame6 transform (ur_kin convention).
// Used by the caller to calibrate the frame6<->tool0 mapping from the live model.
inline Eigen::Isometry3d forward(const double q[6]) {
    double T[16];
    detail::forward(q, T);
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c)
            iso.matrix()(r, c) = T[r*4 + c];
    return iso;
}

// Analytical IK: returns up to 8 solutions. T is base->frame6 (ur_kin convention).
// Solutions are NOT filtered by joint limits (caller validates).
inline std::vector<IKSolution> solve(const Eigen::Isometry3d& T, double q6_des = 0.0) {
    double Tarr[16];
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c)
            Tarr[r*4 + c] = T.matrix()(r, c);
    Tarr[12] = 0.0; Tarr[13] = 0.0; Tarr[14] = 0.0; Tarr[15] = 1.0;

    double q_sols[8*6];
    int n = detail::inverse(Tarr, q_sols, q6_des);
    std::vector<IKSolution> out;
    out.reserve(n);
    for (int s = 0; s < n; ++s) {
        IKSolution sol;
        bool ok = true;
        for (int j = 0; j < 6; ++j) {
            sol.joints[j] = q_sols[s*6 + j];
            if (std::isnan(sol.joints[j])) { ok = false; break; }
        }
        if (ok) out.push_back(sol);
    }
    return out;
}

}  // namespace ur16e_ik

#endif // UR16E_ANALYTICAL_IK_HPP
