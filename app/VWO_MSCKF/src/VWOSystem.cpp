#include <VWO/VWOSystem.h>

#include <assert.h>
#include <glog/logging.h>

#include <VWO/ParamLoader.h>
#include <VWO/StateAugmentor.h>

namespace VWO {

long int VWOSystem::kFrameId = -1;

VWOSystem::VWOSystem(const std::string& param_file) : initialized_(false) {
    /// Load parameters.
    LoadParam(param_file, &param_);

    /// Initialize all modules.
    data_sync_ = std::make_unique<TGK::DataSynchronizer::WheelImageSynchronizer>();

    initializer_ = std::make_unique<Initializer>(param_.extrinsic.O_R_C, param_.extrinsic.O_p_C, 
        param_.wheel_param.kl, param_.wheel_param.kr, param_.wheel_param.b);

    propagator_ = std::make_unique<Propagator>(param_.wheel_param.kl, param_.wheel_param.kr, param_.wheel_param.b, 
                                               param_.wheel_param.noise_factor);

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
        initializer_->Initialize(img_ptr->timestamp, &state_);
        initialized_ = true;
        return true;
    }

    // Propagate state.
    for (size_t i = 1; i < wheel_data_segment.size(); ++i) {
        const auto begin_wheel = wheel_data_segment[i-1];
        const auto end_wheel = wheel_data_segment[i];

        // Check timestamp.
        assert(std::abs(begin_wheel->timestamp - state_.timestamp) < 1e-6);

        propagator_->Propagate(begin_wheel->left, begin_wheel->right,
                               end_wheel->left, end_wheel->right,
                               &state_);

        // Set timestamp.
        state_.timestamp = end_wheel->timestamp;
    }
    
    // Augment state / Clone new camera state.
    AugmentState(img_ptr->timestamp, (++kFrameId), &state_);

    // Update state.

    /// Visualize.
    viz_->DrawWheelPose(state_.wheel_pose.G_R_O, state_.wheel_pose.G_p_O);

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