// FR3 (Franka Research 3) per-joint kinematic limits, sourced from the
// official URDF and FCI documentation. Single source of truth for the
// values referenced by controllers (joint_impedance, osc_impedance) and
// trajectory interpolators (MotionGenerator, SmoothJointTrajInterpolator).

#ifndef UTILS_FR3_CONSTANTS_H_
#define UTILS_FR3_CONSTANTS_H_

#include <Eigen/Core>

namespace fr3 {

inline const Eigen::Array<double, 7, 1> kJointMax =
    (Eigen::Array<double, 7, 1>() << 2.3093, 1.5133, 2.4937, -0.4461, 2.4800,
     4.2094, 2.6895)
        .finished();

inline const Eigen::Array<double, 7, 1> kJointMin =
    (Eigen::Array<double, 7, 1>() << -2.3093, -1.5133, -2.4937, -2.7478, -2.4800,
     0.8521, -2.6895)
        .finished();

inline const Eigen::Matrix<double, 7, 1> kDqMax =
    (Eigen::Matrix<double, 7, 1>() << 2.0, 1.0, 1.5, 1.25, 3.0, 1.5, 3.0)
        .finished();

}  // namespace fr3

#endif  // UTILS_FR3_CONSTANTS_H_
