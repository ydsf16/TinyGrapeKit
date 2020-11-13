#include <VWO/VWOSystem.h>

#include <glog/logging.h>

namespace VWO {

VWOSystem::VWOSystem(const std::string& param_file) : initialized_(false) {
    // TODO: Load parameters.

    /// 2. Initialize all modules.
    data_sync_ = std::make_unique<TGK::DataSynchronizer::WheelImageSynchronizer>();
    initializer_ = std::make_unique<Initializer>(param_.extrinsic.O_R_C, param_.extrinsic.O_p_C, 
        param_.wheel_param.kl, param_.wheel_param.kr, param_.wheel_param.b);
    viz_ = std::make_unique<Visualizer>(param_.viz_config);
}

bool VWOSystem::FeedWheelData(const double timestamp, const double left, const double right) {
    // Convert to internal struct.
    const TGK::BaseType::WheelDataPtr wheel_ptr = std::make_shared<TGK::BaseType::WheelData>();
    wheel_ptr->timestamp = timestamp;
    wheel_ptr->left = left;
    wheel_ptr->right = right;

    // Sync with image data.
    std::vector<TGK::BaseType::WheelDataConstPtr> wheel_data_segment;
    TGK::BaseType::MonoImageDataConstPtr img_ptr;
    if (!data_sync_->FeedWheelData(wheel_ptr, &wheel_data_segment, &img_ptr)) { 
        return true;
    }

    // Initialize.
    if (!initialized_) {
        initializer_->Initialize(timestamp, &state_);
    }

    // Propagate state.
    
    // Track feature.

    // Agument state.

    // Update state.

    /// Visualize.
    viz_->DrawWheelPose(state_.wheel_pose_.G_R_O, state_.wheel_pose_.G_p_O);

    return true;
}
    
bool VWOSystem::FeedImageData(const double timestamp, const cv::Mat& image) {
    // Convert to internal struct.
    const TGK::BaseType::MonoImageDataPtr img_ptr = std::make_shared<TGK::BaseType::MonoImageData>();
    img_ptr->timestamp = timestamp;
    img_ptr->image = image;

    // Sync with wheel data.
    data_sync_->FeedMonoImageData(img_ptr);
    return true;
}

bool VWOSystem::GetCameraPoses(std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>* cam_poses) {
    
    return true;
}

bool VWOSystem::GetWheelPose(Eigen::Matrix3d* G_R_O, Eigen::Vector3d* G_p_O) {
        
    return true;
}

}  // namespace VWO