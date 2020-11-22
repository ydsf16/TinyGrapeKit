#include <VWO/Propagator.h>

#include <glog/logging.h>

namespace VWO {

Propagator::Propagator(const double kl, const double kr,const double b, const double noise_factor) {
    wheel_propagator_ = std::make_unique<TGK::WheelProcessor::WheelPropagator>(kl, kr, b, noise_factor);
}

void Propagator::Propagate(const double begin_wl, const double begin_wr,
                           const double end_wl, const double end_wr,
                           State* state) {
    // Propagate mean and covariance of the wheel pose state.
    Eigen::Matrix<double, 6, 6> J_wrt_wheel_pose;
    Eigen::Matrix<double, 6, 3> J_wrt_intrinsic;
    Eigen::Matrix<double, 6, 6> wheel_pose_cov = state->covariance.topLeftCorner<6, 6>();
    wheel_propagator_->PropagateUsingEncoder(state->wheel_intrinsic.kl, state->wheel_intrinsic.kr, state->wheel_intrinsic.b,
                                             begin_wl, begin_wr, end_wl, end_wr, 
                                             &state->wheel_pose.G_R_O, &state->wheel_pose.G_p_O,
                                             &J_wrt_wheel_pose,
                                             &J_wrt_intrinsic,
                                             &wheel_pose_cov);
    // Add intrinsic related term.
    const Eigen::Matrix<double, 6, 3> cov_ts = state->covariance.block<6, 3>(0, 6);
    const Eigen::Matrix<double, 3, 6> cov_st = cov_ts.transpose();
    const Eigen::Matrix<double, 3, 3> cov_ss = state->covariance.block<3, 3>(6, 6);
    state->covariance.topLeftCorner<6, 6>() = wheel_pose_cov +
        (J_wrt_intrinsic * cov_st * J_wrt_wheel_pose.transpose() +
        J_wrt_wheel_pose * cov_ts * J_wrt_intrinsic.transpose() +
        J_wrt_intrinsic * cov_ss * J_wrt_intrinsic.transpose());
   
    state->covariance.block<6, 3>(0, 6) = J_wrt_wheel_pose * cov_ts  + J_wrt_intrinsic * cov_ss;
    state->covariance.block<3, 6>(6, 0) = state->covariance.block<6, 3>(0, 6).eval().transpose();

    state->covariance(state->wheel_intrinsic.state_idx, state->wheel_intrinsic.state_idx) += klkr_process_noise_;
    state->covariance(state->wheel_intrinsic.state_idx + 1, state->wheel_intrinsic.state_idx + 1) += klkr_process_noise_;
    state->covariance(state->wheel_intrinsic.state_idx + 2, state->wheel_intrinsic.state_idx + 2) += b_process_noise_;

    // Propagate covariance of other states.
    const int cov_size = state->covariance.rows();
    const int other_size = cov_size - state->slid_idx;
    if (other_size <= 0) { return; }

    // Big Phi
    Eigen::MatrixXd Phi(state->slid_idx, state->slid_idx);
    Phi.setIdentity();
    Phi.block<6, 6>(0, 0) = J_wrt_wheel_pose;
    Phi.block<6, 3>(0, 6) = J_wrt_intrinsic;

    state->covariance.block(0, state->slid_idx, state->slid_idx, other_size) = 
        Phi * state->covariance.block(0, state->slid_idx, state->slid_idx, other_size).eval();

    //Force symmetric.
    state->covariance = state->covariance.eval().selfadjointView<Eigen::Upper>();
}

}  // namespace VWO