#pragma once

#include <deque>

#include <glog/logging.h>

#include <TGK/Const/Const.h>
#include <VWO/StateBlock.h>

namespace VWO {

struct State {
    double timestamp;

    // Wheel Instrinsic.
    WheelIntrinsic wheel_intrinsic;
    // Extrinsic.
    Extrinsic extrinsic;
    // The current wheel pose.
    WheelPose wheel_pose;
    // Camera poses.
    std::deque<CameraFramePtr> camera_frames;

    // Covariance.
    Eigen::MatrixXd covariance;

    void Update(const Eigen::VectorXd& delta_x) {
        constexpr double kMaxDeltaRot = 5. * kDeg2Rad; // rad.
        constexpr double kMaxDeltaTrans = 0.5; // meter.

        const Eigen::Vector3d delta_ang = delta_x.segment(wheel_pose.state_idx, 3);
        if (delta_ang.norm() > kMaxDeltaRot) { 
            LOG(ERROR) << "Delta rot of wheel pose is too large. " << std::fixed << delta_ang.norm();
            return; 
        }

        const Eigen::Vector3d delta_pos = delta_x.segment(wheel_pose.state_idx + 3, 3);
        if (delta_pos.norm() > kMaxDeltaTrans) { 
            LOG(ERROR) << "Delta trans of wheel pose is too large. " << std::fixed << delta_pos.norm();
            return; 

        }

        for (CameraFramePtr& cam_fm : camera_frames) {
            const Eigen::Vector3d delta_ang = delta_x.segment(cam_fm->state_idx, 3);
            if (delta_ang.norm() > kMaxDeltaRot) { 
                LOG(ERROR) << "Delta rot of camera pose is too large. " << std::fixed << delta_ang.norm();
                return; 
            }

            const Eigen::Vector3d delta_pos = delta_x.segment(cam_fm->state_idx + 3, 3);
            if (delta_pos.norm() > kMaxDeltaTrans) { 
                LOG(ERROR) << "Delta trans of camera pose is too large. " << std::fixed << delta_pos.norm();
                return; 
            }
        }

        // Update Wheel_pose.
        wheel_pose.Update(delta_x.segment(wheel_pose.state_idx, wheel_pose.size));

        // Update Camera pose.
        for (CameraFramePtr& cam_fm : camera_frames) {
            cam_fm->Update(delta_x.segment(cam_fm->state_idx, cam_fm->size));
        }
    }
};

}  // namespace VWO