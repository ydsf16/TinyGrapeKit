#pragma once

#include <Eigen/Eigen>

#include "Utils.h"

namespace SINS {

inline Eigen::Matrix3d SO3Exp(const Eigen::Vector3d &v) {
    double theta = v.norm();
    if (theta < 1e-14) {
        return Eigen::Matrix3d::Identity() + SkewMat(v);
    }

    return Eigen::AngleAxisd(theta, v.normalized()).toRotationMatrix();
} 

inline Eigen::Matrix3d NormalizeRotMat(const Eigen::Matrix3d &rot) {
    Eigen::Quaterniond quat(rot);
    quat.normalize();
    return quat.toRotationMatrix();
}

Eigen::Vector3d RotMatToEuler(const Eigen::Matrix3d &C) {
    double roll = std::atan2(C(2, 1), C(2, 2));
    double pitch = std::atan2(-C(2, 0), sqrt(C(2, 1)* C(2, 1) + C(2, 2) * C(2, 2)));
    double yaw = std::atan2(C(1, 0), C(0, 0));
    return Eigen::Vector3d(yaw, pitch, roll);
}

} // namespace SINS