#include <WheelProcessor/WheelPropagator.h>

#include <Eigen/Geometry>

#include <Util/Util.h>

namespace TGK {
namespace WheelProcessor {

WheelPropagator::WheelPropagator(const double kl, const double kr, const double b, 
                                 const double noise_factor, const double roll_pitch_noise, const double z_noise) 
    : kl_(kl), kr_(kr), b_(b), noise_factor_(noise_factor), roll_pitch_noise_(roll_pitch_noise), z_noise_(z_noise) { }

void WheelPropagator::PropagateUsingEncoder(const double kl, const double kr, const double b,
                                            const double begin_wl, const double begin_wr,
                                            const double end_wl, const double end_wr,
                                            Eigen::Matrix3d* G_R_O, Eigen::Vector3d* G_p_O,
                                            Eigen::Matrix<double, 6, 6>* J_wrt_pose,
                                            Eigen::Matrix<double, 6, 3>* J_wrt_intrinsic,
                                            Eigen::Matrix<double, 6, 6>* cov) {
    kl_ = kl;
    kr_ = kr;
    b_ = b;

    PropagateUsingEncoder(begin_wl, begin_wr, end_wl, end_wr, G_R_O, G_p_O, J_wrt_pose, J_wrt_intrinsic, cov);
}

void WheelPropagator::PropagateUsingEncoder(const double begin_wl, const double begin_wr,
                                            const double end_wl, const double end_wr,
                                            Eigen::Matrix3d* G_R_O, Eigen::Vector3d* G_p_O,
                                            Eigen::Matrix<double, 6, 6>* J_wrt_pose,
                                            Eigen::Matrix<double, 6, 3>* J_wrt_intrinsic,
                                            Eigen::Matrix<double, 6, 6>* cov) {
    // Pre-computation.
    const double left_w_dist = (end_wl - begin_wl);
    const double left_dist = left_w_dist * kl_;
    const double right_w_dist = (end_wr - begin_wr);
    const double right_dist = right_w_dist * kr_;
    const double delta_yaw = (right_dist - left_dist) / b_;
    const double delta_dist = (right_dist + left_dist) * 0.5;

    // Mean.
    const Eigen::Matrix3d delta_R = Eigen::AngleAxisd(delta_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d delta_p = Eigen::Vector3d(delta_dist, 0., 0.);
    const Eigen::Matrix3d old_G_R_O = *G_R_O;

    *G_R_O = old_G_R_O * delta_R;
    *G_p_O = *G_p_O + old_G_R_O * delta_p;

    // Jacobian or Phi.
    if (J_wrt_pose != nullptr) {
        *J_wrt_pose << delta_R.transpose(),            Eigen::Matrix3d::Zero(), 
                      -old_G_R_O *Util::Skew(delta_p), Eigen::Matrix3d::Identity();
    }

    // Jacobian WRT intrinsic.
    if (J_wrt_intrinsic != nullptr) {
        Eigen::Matrix<double, 6, 6> J_wrt_delta_pose = Eigen::Matrix<double, 6, 6>::Identity();
        J_wrt_delta_pose.block<3, 3>(3, 3) = old_G_R_O;

        Eigen::Matrix<double, 6, 3> J_delta_pose_wrt_intrinsic = Eigen::Matrix<double, 6, 3>::Zero();
        J_delta_pose_wrt_intrinsic(2, 0) = -left_w_dist / b_;
        J_delta_pose_wrt_intrinsic(2, 1) = right_w_dist / b_;
        J_delta_pose_wrt_intrinsic(2, 2) = -(right_dist - left_dist) / (b_ * b_);

        J_delta_pose_wrt_intrinsic(3, 0) = left_w_dist * 0.5;
        J_delta_pose_wrt_intrinsic(3, 1) = right_w_dist * 0.5;
   
        *J_wrt_intrinsic = J_wrt_delta_pose * J_delta_pose_wrt_intrinsic;
    }

    // Covariance.
    if (cov != nullptr && J_wrt_pose != nullptr) {
        Eigen::Matrix<double, 6, 6> noise = Eigen::Matrix<double, 6, 6>::Zero();
        noise(0, 0) = roll_pitch_noise_;
        noise(1, 1) = roll_pitch_noise_;
        noise(2, 2) = delta_yaw * delta_yaw * noise_factor_ * noise_factor_;
        noise(3, 3) = delta_dist * delta_dist * noise_factor_ * noise_factor_;
        noise(4, 4) = delta_dist * delta_dist * noise_factor_ * noise_factor_;
        noise(5, 5) = z_noise_;

        Eigen::Matrix<double, 6, 6> J_wrt_delta_pose = Eigen::Matrix<double, 6, 6>::Identity();
        J_wrt_delta_pose.block<3, 3>(3, 3) = old_G_R_O;

        *cov = (*J_wrt_pose) * cov->eval() * J_wrt_pose->transpose() + 
               J_wrt_delta_pose * noise * J_wrt_delta_pose.transpose();
    }
}

}  // namespace WheelProcessor
}  // namespace TGK