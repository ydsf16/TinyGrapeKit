#include <FilterFusion/IMUUpdater.h>

#include <glog/logging.h>

#include <basalt/spline/so3_spline.h>

namespace FilterFusion {

IMUUpdater::IMUUpdater(const Eigen::Matrix3d& C_R_I, const Eigen::Vector3d& C_p_I, const Config& config) 
    : C_R_I_(C_R_I), C_p_I_(C_p_I), config_(config) { }

bool IMUUpdater::UpdateState(const TGK::BaseType::IMUDataConstPtr imu_data, State* state) {
    // Save imu data to buffer.
    imu_buffer_.push_back(imu_data);

    // Check clone size.
    if (state->camera_frames.size() < 4) {
        LOG(WARNING) << "[UpdateState]: No enought clones";
        return false;
    }

    int num_clone = state->camera_frames.size();
    CameraFramePtr cam1 = state->camera_frames[num_clone-4];
    CameraFramePtr cam2 = state->camera_frames[num_clone-3];
    CameraFramePtr cam3 = state->camera_frames[num_clone-2];
    CameraFramePtr cam4 = state->camera_frames[num_clone-1];

    // Erase all imus before cam2.
    while (!imu_buffer_.empty() && imu_buffer_.front()->timestamp <= cam2->timestamp) {
        imu_buffer_.pop_front();
    }

    // Check if all data arrived between cam2 and cam3.
    if (imu_buffer_.empty() || 
        imu_buffer_.back()->timestamp < cam3->timestamp || 
        imu_buffer_.front()->timestamp >= cam3->timestamp) {
        LOG(INFO) << "[UpdateState]: Watting data.";
        return true;
    }

    // Collect all imu data in cam2 and cam3.
    std::vector<TGK::BaseType::IMUDataConstPtr> imu_vec;
    for (const auto imu_data : imu_buffer_) {
        if (imu_data->timestamp > cam3->timestamp) { break; }
        if (imu_data->timestamp >= cam2->timestamp && imu_data->timestamp <= cam3->timestamp) {
            imu_vec.push_back(imu_data);
        }
    }

    // Update using IMU.
    

    // Remove all used ims.
    while (!imu_buffer_.empty() && imu_buffer_.front()->timestamp <= cam3->timestamp) {
        imu_buffer_.pop_front();
    }
    
    return true;
}

bool IMUUpdater::UpdateUsingIMUVector(const std::vector<TGK::BaseType::IMUDataConstPtr>& imu_vec,
                                      GyroBias* gyro_bias, AccBias* acc_bias, 
                                      CameraFrame* cam1, CameraFrame* cam2, CameraFrame* cam3, CameraFrame* cam4) {
    // First create the B-Spline.

    return true;
}

}  // namespace FilterFusion