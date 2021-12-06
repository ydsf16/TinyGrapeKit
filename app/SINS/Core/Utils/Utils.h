#pragma once

#include <Eigen/Eigen>

namespace SINS {

inline Eigen::Matrix3d SkewMat(const Eigen::Vector3d &v) {
    Eigen::Matrix3d skew_mat;

    skew_mat << 0.0, -v.z(), v.y(),
                v.z(), 0.0, -v.x(),
                -v.y(), v.x(), 0.0;

    return skew_mat;
}

}  // namespace SINS