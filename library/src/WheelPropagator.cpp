#include <WheelProcessor/WheelPropagator.h>

#include <Eigen/Geometry>

#include <Util/Util.h>

namespace TGK {
namespace WheelProcessor {

WheelPropagator::WheelPropagator(const double kl, const double kr, const double b, 
                                 const double noise_factor, const double roll_pitch_noise, const double z_noise) 
    : kl_(kl), kr_(kr), b_(b), noise_factor_(noise_factor), roll_pitch_noise_(roll_pitch_noise), z_noise_(z_noise) { }

void WheelPropagator::PropagateUsingEncoder(const double begin_wl, const double begin_wr,
                                            const double end_wl, const double end_wr,
                                            Eigen::Matrix3d* G_R_O, Eigen::Vector3d* G_p_O,
                                            Eigen::Matrix<double, 6, 6>* J_wrt_pose,
                                            Eigen::Matrix<double, 6, 6>* cov) {
    // Pre-computation.
    const double left_dist = (end_wl - begin_wl) * kl_;
    const double right_dist = (end_wr - begin_wr) * kr_;
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

    // Covariance.
    if (cov != nullptr && J_wrt_pose != nullptr) {
        Eigen::Matrix<double, 6, 2> J_wrt_lr_dist;
        J_wrt_lr_dist << Eigen::Vector3d(0., 0., -1. / b_),         Eigen::Vector3d(0., 0., 1. / b_),
                         old_G_R_O * Eigen::Vector3d(0.5, 0., 0.), old_G_R_O * Eigen::Vector3d(0.5, 0., 0.);
        
        const Eigen::Matrix2d dist_noise = 
            (Eigen::Matrix2d() << left_dist * noise_factor_, 0., 
                                  0.,                        right_dist * noise_factor_).finished();

        *cov = (*J_wrt_pose) * cov->eval() * J_wrt_pose->transpose() + 
               J_wrt_lr_dist * dist_noise * J_wrt_lr_dist.transpose();

        // Additive noise for roll pitch and　z.
        (*cov)(0, 0) += roll_pitch_noise_;
        (*cov)(1, 1) += roll_pitch_noise_;
        (*cov)(2, 2) += z_noise_;
    }
}

}  // namespace WheelProcessor
}  // namespace TGK