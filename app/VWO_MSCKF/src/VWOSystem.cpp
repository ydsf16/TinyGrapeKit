#include <VWO/VWOSystem.h>

#include <glog/logging.h>

namespace VWO {

VWOSystem::VWOSystem(const std::string& param_file) { }

bool VWOSystem::FeedIMU(const double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro) {
    
    return true;
}
    
bool VWOSystem::FeedImage(const double timestamp, const cv::Mat& image) {

    return true;
}

bool VWOSystem::GetCameraPoses(std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>* cam_poses) {
    
    return true;
}

bool VWOSystem::GetWheelPose(Eigen::Matrix3d* G_R_O, Eigen::Vector3d* G_p_O) {
        
    return true;
}

}  // namespace VWO