#include "VWO/Visualizer.h"

namespace VWO {

Visualizer::Visualizer(const Config& config) : config_(config) { }

void Visualizer::DrawCameras(const std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>& camera_poses) {
    std::lock_guard<std::mutex> lg(data_buffer_mutex_);
    camera_poses_ = camera_poses;
}

void Visualizer::DrawWheelPose(const Eigen::Matrix3d& G_R_O, const Eigen::Vector3d& G_p_O) {
    std::lock_guard<std::mutex> lg(data_buffer_mutex_);
    wheel_traj_.emplace_back(G_R_O, G_p_O);
    if (wheel_traj_.size() > config_.max_traj_length) { wheel_traj_.pop_front(); }
}

void Visualizer::DrawFeatures(const std::vector<Eigen::Vector3d>& features) {
    std::lock_guard<std::mutex> lg(data_buffer_mutex_);
    for (const Eigen::Vector3d& ft : features) {
        features_.push_back(ft);
    }
    while (features_.size() > config_.max_num_features) { features_.pop_front(); } 
}

void Visualizer::Run() {
    // Config panglin.
    

}

}  // namespace VWO