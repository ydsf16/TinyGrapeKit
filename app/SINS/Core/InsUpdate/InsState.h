#pragma once

#include <Eigen/Eigen>

namespace SINS {

struct InsState {
    double time;  // [s]

    // PVA.
    Eigen::Vector3d lon_lat_hei; // [rad, rad, m] 
    Eigen::Vector3d velocity;    // v_nb
    Eigen::Matrix3d orientation; // C_nb.

    // IMU params.
    Eigen::Vector3d gyro_bias;
    Eigen::Vector3d acc_bias;

    // Earth params.
    bool update_earth = false;
    Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.8);
    double Rm;
    double Rn;
    Eigen::Vector3d wie; 
    Eigen::Vector3d wen; 
    Eigen::Vector3d win; 
};

}  // namespace SINS