#pragma once

#include <deque>

#include <VWO/StateBlock.h>

namespace VWO {

struct State {
    double timestamp;

    // Extrinsic.
    Extrinsic extrinsic_;
    // The current wheel pose.
    WheelPose wheel_pose_;
    // Camera poses.
    std::deque<CameraFrame> camera_frame_;

    // Covariance.
    Eigen::MatrixXd covariance_;

    void Update(const Eigen::VectorXd& delta_x) {
        // Update Wheel_pose.
        wheel_pose_.Update(delta_x.segment<6>(wheel_pose_.state_idx));

        // Update Camera pose.
        for (CameraFrame& cam_fm : camera_frame_) {
            cam_fm.Update(delta_x.segment<6>(cam_fm.state_idx));
        }
    }
};

}  // namespace VWO