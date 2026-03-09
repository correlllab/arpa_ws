#ifndef UR16E_ANALYTICAL_IK_HPP
#define UR16E_ANALYTICAL_IK_HPP

#include <Eigen/Geometry>
#include <vector>
#include <cmath>
#include <array>

namespace ur16e_ik {

struct IKSolution {
    double joints[6];  // shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
};

// UR16e DH parameters
namespace dh {
    constexpr double d1 = 0.1807;
    constexpr double a2 = -0.4784;
    constexpr double a3 = -0.36;
    constexpr double d4 = 0.17415;
    constexpr double d5 = 0.11985;
    constexpr double d6 = 0.11655;
}

inline double normalize_angle(double a) {
    a = std::fmod(a, 2.0 * M_PI);
    if (a > M_PI) a -= 2.0 * M_PI;
    if (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// Returns up to 8 analytical IK solutions for a given target pose
// in the UR base frame. Solutions are NOT filtered by joint limits.
inline std::vector<IKSolution> solve(const Eigen::Isometry3d& T) {
    using namespace dh;

    std::vector<IKSolution> solutions;
    solutions.reserve(8);

    const Eigen::Matrix3d& R06 = T.rotation();
    const Eigen::Vector3d& p06 = T.translation();

    // Wrist center: p05 = p06 - d6 * z_axis_of_tool
    Eigen::Vector3d p05 = p06 - d6 * R06.col(2);

    // ===== theta1 (shoulder pan): 2 solutions =====
    double psi = std::atan2(p05.y(), p05.x());
    double r_xy = std::hypot(p05.x(), p05.y());
    if (r_xy < 1e-10) return solutions;

    double acos_arg1 = d4 / r_xy;
    if (std::abs(acos_arg1) > 1.0) return solutions;
    double phi1 = std::acos(acos_arg1);

    std::array<double, 2> theta1_vals = {
        normalize_angle(psi + phi1 + M_PI / 2.0),
        normalize_angle(psi - phi1 + M_PI / 2.0)
    };

    for (double q1 : theta1_vals) {
        double s1 = std::sin(q1), c1 = std::cos(q1);

        // ===== theta5 (wrist 2): 2 solutions per theta1 =====
        double cos5 = (p06.x() * s1 - p06.y() * c1 - d4) / d6;
        if (std::abs(cos5) > 1.0 + 1e-8) continue;
        cos5 = std::clamp(cos5, -1.0, 1.0);

        std::array<double, 2> theta5_vals = { std::acos(cos5), -std::acos(cos5) };

        for (double q5 : theta5_vals) {
            double s5 = std::sin(q5);

            // ===== theta6 (wrist 3): 1 solution per (q1, q5) =====
            double q6;
            if (std::abs(s5) < 1e-10) {
                q6 = 0.0;  // Singularity — theta6 arbitrary
            } else {
                double sin6 = (-R06(0, 1) * s1 + R06(1, 1) * c1) / s5;
                double cos6 = ( R06(0, 0) * s1 - R06(1, 0) * c1) / s5;
                q6 = std::atan2(sin6, cos6);
            }

            // ===== theta2, theta3 (shoulder lift + elbow): 2 solutions =====
            // Transform wrist center to shoulder plane (frame 1 XZ plane)
            // In frame 1, the relevant coordinates are:
            // p14_x = c1*p05.x + s1*p05.y    (along the arm plane)
            // p14_z = p05.z - d1              (vertical in frame 1)
            double p1x = c1 * p05.x() + s1 * p05.y();
            double p1z = p05.z() - d1;

            // 2R problem with d5 offset:
            //   a2*c2 + (a3*c23 + d5*s23) = p1x
            //   a2*s2 + (a3*s23 - d5*c23) = p1z
            // Let A' = sqrt(a3^2 + d5^2), phi' = atan2(d5, a3)
            // Then the effective second link rotates theta23 by phi'
            double Ap = std::hypot(a3, d5);
            double phip = std::atan2(d5, a3);

            double r_sq = p1x * p1x + p1z * p1z;
            // cos(theta3 + phip) = (r^2 - a2^2 - Ap^2) / (2 * a2 * Ap)
            double cos3p = (r_sq - a2 * a2 - Ap * Ap) / (2.0 * a2 * Ap);
            if (std::abs(cos3p) > 1.0 + 1e-8) continue;
            cos3p = std::clamp(cos3p, -1.0, 1.0);

            std::array<double, 2> q3p_vals = { std::acos(cos3p), -std::acos(cos3p) };

            for (double q3p : q3p_vals) {
                double q3 = normalize_angle(q3p - phip);
                double s3p = std::sin(q3p);

                // theta2: standard 2R solution
                // k1 = a2 + Ap*cos(q3p),  k2 = Ap*sin(q3p)
                double k1 = a2 + Ap * cos3p;
                double k2 = Ap * s3p;
                double q2 = std::atan2(p1z, p1x) - std::atan2(k2, k1);
                q2 = normalize_angle(q2);

                // ===== theta4 (wrist 1) =====
                // Build R03 from DH: R01*R12*R23
                double c23 = std::cos(q2 + q3), s23 = std::sin(q2 + q3);
                Eigen::Matrix3d R03;
                R03 << c1*c23, -c1*s23,  s1,
                       s1*c23, -s1*s23, -c1,
                       s23,     c23,     0.0;

                Eigen::Matrix3d R36 = R03.transpose() * R06;

                // From R36 = R34*R45*R56:
                // R36(0,2) = c4*s5, R36(1,2) = s4*s5
                double q4;
                if (std::abs(s5) < 1e-10) {
                    q4 = 0.0;
                } else {
                    q4 = std::atan2(R36(1, 2) / s5, R36(0, 2) / s5);
                }
                q4 = normalize_angle(q4);

                // Validate: skip NaN solutions
                IKSolution sol;
                sol.joints[0] = normalize_angle(q1);
                sol.joints[1] = normalize_angle(q2);
                sol.joints[2] = normalize_angle(q3);
                sol.joints[3] = normalize_angle(q4);
                sol.joints[4] = normalize_angle(q5);
                sol.joints[5] = normalize_angle(q6);

                bool valid = true;
                for (int i = 0; i < 6; ++i) {
                    if (std::isnan(sol.joints[i])) { valid = false; break; }
                }
                if (!valid) continue;

                solutions.push_back(sol);
            }
        }
    }

    return solutions;
}

}  // namespace ur16e_ik

#endif // UR16E_ANALYTICAL_IK_HPP
