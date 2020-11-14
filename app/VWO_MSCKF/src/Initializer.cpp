#include <VWO/Initializer.h>

namespace VWO {

Initializer::Initializer(const Eigen::Matrix3d& O_R_C, const Eigen::Vector3d& O_p_C, 
                        const double kl, const double kr, const double b) 
    : O_R_C_(O_R_C), O_p_C_(O_p_C), kl_(kl), kr_(kr), b_(b) { }

void Initializer::Initialize(const double timestamp, State* init_state) {
    init_state->timestamp = timestamp;

    // Do not estimate extrinsic.
    init_state->extrinsic_.O_R_C = O_R_C_;
    init_state->extrinsic_.O_p_C = O_p_C_;

    // Do not estimate intrinsic.
    init_state->wheel_intrinsic_.kl = kl_;
    init_state->wheel_intrinsic_.kr = kr_;
    init_state->wheel_intrinsic_.b = b_;

    // Set initial wheel pose to ZERO.
    int state_idx = 0;
    init_state->wheel_pose_.G_R_O.setIdentity();
    init_state->wheel_pose_.G_p_O.setZero();
    init_state->wheel_pose_.state_idx = state_idx;

    // Clear camera frame.
    init_state->camera_frame_.clear();

    // Set initial covariance.
    const int cov_size = init_state->wheel_pose_.size;  
    init_state->covariance_.resize(cov_size, cov_size);
    init_state->covariance_.setIdentity();
    init_state->covariance_ *= 1e-12;
}

}  // namespace VWO