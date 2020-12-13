#pragma once

#include <Eigen/Core>

#include <FilterFusion/State.h>
#include <TGK/BaseType/Measurement.h>

namespace FilterFusion {

class IMUUpdater {
public:
    struct Config {
        double acc_noise = 1e-8;
        double gyro_noise = 1e-12;
        double gravity = 9.8015; // m/s^2
    };

    IMUUpdater(const Eigen::Matrix3d& C_R_I, const Eigen::Vector3d& C_p_I, const Config& config);

    bool UpdateState(const TGK::BaseType::IMUDataConstPtr imu_data, State* state);

private:
    bool UpdateUsingIMUVector(const std::vector<TGK::BaseType::IMUDataConstPtr>& imu_vec,
                              GyroBias* gyro_bias, AccBias* acc_bias, 
                              CameraFrame* cam1, CameraFrame* cam2, CameraFrame* cam3, CameraFrame* cam4);
                          
    const Config config_;
    Eigen::Matrix3d C_R_I_;
    Eigen::Vector3d C_p_I_;

    std::deque<TGK::BaseType::IMUDataConstPtr> imu_buffer_;
};  

}  // namespace FilterFusion