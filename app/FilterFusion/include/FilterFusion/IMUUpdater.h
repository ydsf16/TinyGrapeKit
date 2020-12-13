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

    IMUUpdater(const Config& config);

    bool UpdateState(const TGK::BaseType::IMUDataConstPtr imu_data, State* state);

private:
    const Config config_;

    std::deque<TGK::BaseType::IMUDataConstPtr> imu_buffer_;
};  

}  // namespace FilterFusion