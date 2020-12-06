#pragma once

#include <deque>

#include <Eigen/Core>

#include <VWO/State.h>
#include <TGK/BaseType/Measurement.h>

namespace VWO {

class GpsUpdater {
public:
    GpsUpdater(const Eigen::Vector3d& C_p_Gps);

    bool UpdateState(const TGK::BaseType::GpsDataConstPtr gps_data, State* state);
    
    void SetInitLonLatHei(const Eigen::Vector3d& init_lon_lat_hei);

private:
    Eigen::Vector3d C_p_Gps_;
    
    Eigen::Vector3d init_lon_lat_hei_;
    bool init_set = false;
    
    std::deque<TGK::BaseType::GpsDataConstPtr> gps_data_queue_;
};

}  // namespace VWO