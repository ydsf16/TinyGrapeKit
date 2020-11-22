#include <VWO/Initializer.h>

namespace VWO {

Initializer::Initializer(const Eigen::Matrix3d& O_R_C, const Eigen::Vector3d& O_p_C, 
                        const double kl, const double kr, const double b) 
    : O_R_C_(O_R_C), O_p_C_(O_p_C), kl_(kl), kr_(kr), b_(b) { }

void Initializer::Initialize(const double timestamp, State* init_state) {
    init_state->timestamp = timestamp;

    // Set initial wheel pose to ZERO.
    int state_idx = 0;
    init_state->wheel_pose.G_R_O.setIdentity();
    init_state->wheel_pose.G_p_O.setZero();
    init_state->wheel_pose.state_idx = state_idx;
    state_idx += init_state->wheel_pose.size;

    // Do not estimate extrinsic.
    init_state->extrinsic.O_R_C = O_R_C_;
    init_state->extrinsic.O_p_C = O_p_C_;

    // Do not estimate intrinsic.
    init_state->wheel_intrinsic.kl = kl_;
    init_state->wheel_intrinsic.kr = kr_;
    init_state->wheel_intrinsic.b = b_;
    init_state->wheel_intrinsic.state_idx = state_idx;
    state_idx += init_state->wheel_intrinsic.size;

    // Set start index of the sliding window.
    init_state->slid_idx = state_idx;

    // Clear camera frame.
    init_state->camera_frames.clear();

    // Set initial covariance.
    const int cov_size = init_state->slid_idx;  
    init_state->covariance.resize(cov_size, cov_size);
    init_state->covariance.setIdentity();
    init_state->covariance *= 1e-12;
}

}  // namespace VWO