#include <FilterFusion/IMUUpdater.h>

#include <glog/logging.h>

#include <basalt/spline/so3_spline.h>
#include <basalt/spline/rd_spline.h>
#include <TGK/Util/Util.h>
#include <FilterFusion/UpdaterUtil.h>

#include <fstream>

namespace FilterFusion {

constexpr double kSecToNanoSec = 1e9;

void ForceOrthogonal(Eigen::Matrix3d* R) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(*R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    *R = U *V.transpose();
    if (R->determinant() < 0.) {
        *R = -R->eval();
    }
}

IMUUpdater::IMUUpdater(const Eigen::Matrix3d& C_R_I, const Eigen::Vector3d& C_p_I, const Config& config) 
    : C_R_I_(C_R_I), C_p_I_(C_p_I), config_(config) { }

bool IMUUpdater::UpdateState(const TGK::BaseType::IMUDataConstPtr imu_data, State* state) {
    // Save imu data to buffer.
    imu_buffer_.push_back(imu_data);

    // Check clone size.
    if (state->camera_frames.size() < 4) {
        LOG(WARNING) << "[UpdateState]: No enought clones";
        return false;
    }

    int num_clone = state->camera_frames.size();
    CameraFramePtr cam1 = state->camera_frames[num_clone-4];
    CameraFramePtr cam2 = state->camera_frames[num_clone-3];
    CameraFramePtr cam3 = state->camera_frames[num_clone-2];
    CameraFramePtr cam4 = state->camera_frames[num_clone-1];

    // Erase all imus before cam1.
    while (!imu_buffer_.empty() && imu_buffer_.front()->timestamp <= cam1->timestamp) {
        imu_buffer_.pop_front();
    }

    // Check if all data arrived between cam1 and cam2.
    if (imu_buffer_.empty() || 
        imu_buffer_.back()->timestamp < cam2->timestamp || 
        imu_buffer_.front()->timestamp >= cam2->timestamp) {
        LOG(INFO) << "[UpdateState]: Watting data.";
        return true;
    }

    // Collect all imu data in cam1 and cam2.
    std::vector<TGK::BaseType::IMUDataConstPtr> imu_vec;
    for (const auto imu_data : imu_buffer_) {
        if (imu_data->timestamp >= cam2->timestamp) { break; }
        if (imu_data->timestamp >= cam1->timestamp && imu_data->timestamp < cam2->timestamp) {
            imu_vec.push_back(imu_data);
        }
    }

    // Update using IMU.
    UpdateUsingIMUVector(imu_vec, cam1.get(), cam2.get(), cam3.get(), cam4.get(), state);

    // Remove all used ims.
    while (!imu_buffer_.empty() && imu_buffer_.front()->timestamp < cam2->timestamp) {
        imu_buffer_.pop_front();
    }
    
    return true;
}

bool EvaluateOneJacobian(basalt::So3Spline<4, double>& so3_spline,
                         basalt::RdSpline<3, 4, double>& rd_spline,
                         const Eigen::Vector3d& bg, const Eigen::Vector3d& ba,
                         const TGK::BaseType::IMUData& imu_data,
                         Eigen::Matrix<double, 6, 1>* residual, 
                         Eigen::Matrix<double, 6, 6>* J_wrt_bias,
                         Eigen::Matrix<double, 6, 24>* J_wrt_knots) {
    const uint64_t timestamp = imu_data.timestamp * kSecToNanoSec;
    const Eigen::Vector3d kGravity(0., 0., 9.81);

    // Get omega.
    basalt::So3Spline<4, double>::JacobianStruct so3_vel_Jacobian;
    basalt::So3Spline<4, double>::JacobianStruct so3_Jacobian;
    const Eigen::Vector3d I_w_I = so3_spline.velocityBody(timestamp, &so3_vel_Jacobian);
    const Sophus::SO3<double> so3_G_R_I = so3_spline.evaluate(timestamp, &so3_Jacobian);
    const Eigen::Matrix3d G_R_I = so3_G_R_I.matrix();

    // Get acc.
    basalt::RdSpline<3, 4, double>::JacobianStruct rd_acc_Jacobian;
    basalt::RdSpline<3, 4, double>::JacobianStruct rd_Jacobian;
    const Eigen::Vector3d G_a_I = rd_spline.acceleration(timestamp, &rd_acc_Jacobian);
    const Eigen::Vector3d G_p_I = rd_spline.evaluate(timestamp, &rd_Jacobian);

    // Compute residual.
    residual->head<3>() = imu_data.gyro - (I_w_I + bg);
    const Eigen::Vector3d I_a_I = G_R_I.transpose() * (G_a_I + kGravity);
    residual->segment<3>(3) = imu_data.acc - (I_a_I + ba);

    // Compute Jacobians.
    J_wrt_bias->setIdentity();

    Eigen::Matrix<double, 6, 9> Ht;
    Ht.setZero();
    Ht.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    Ht.block<3, 3>(3, 0) = G_R_I.transpose() * TGK::Util::Skew(G_a_I + kGravity);
    Ht.block<3, 3>(3, 6) = G_R_I.transpose();

    Eigen::Matrix<double, 9, 24> Hs;
    Hs.setZero();
    Hs.block<3, 3>(0, 0) = so3_Jacobian.d_val_d_knot[0];
    Hs.block<3, 3>(0, 6) = so3_Jacobian.d_val_d_knot[1];
    Hs.block<3, 3>(0, 12) = so3_Jacobian.d_val_d_knot[2];
    Hs.block<3, 3>(0, 18) = so3_Jacobian.d_val_d_knot[3];

    Hs.block<3, 3>(3, 0) = so3_vel_Jacobian.d_val_d_knot[0];
    Hs.block<3, 3>(3, 6) = so3_vel_Jacobian.d_val_d_knot[1];
    Hs.block<3, 3>(3, 12) = so3_vel_Jacobian.d_val_d_knot[2];
    Hs.block<3, 3>(3, 18) = so3_vel_Jacobian.d_val_d_knot[3];
    
    Hs.block<3, 3>(6, 3) = rd_acc_Jacobian.d_val_d_knot[0] * Eigen::Matrix3d::Identity();
    Hs.block<3, 3>(6, 9) = rd_acc_Jacobian.d_val_d_knot[1] * Eigen::Matrix3d::Identity();
    Hs.block<3, 3>(6, 15) = rd_acc_Jacobian.d_val_d_knot[2] * Eigen::Matrix3d::Identity();
    Hs.block<3, 3>(6, 21) = rd_acc_Jacobian.d_val_d_knot[3] * Eigen::Matrix3d::Identity();

    *J_wrt_knots = Ht * Hs;

    // Convert Left Jacobian to Right Jacobain.
    J_wrt_knots->block<3, 3>(0, 0) = J_wrt_knots->block<3, 3>(0, 0).eval() * so3_spline.getKnot(0).matrix();
    J_wrt_knots->block<3, 3>(0, 6) = J_wrt_knots->block<3, 3>(0, 6).eval() * so3_spline.getKnot(1).matrix();
    J_wrt_knots->block<3, 3>(0, 12) = J_wrt_knots->block<3, 3>(0, 12).eval() * so3_spline.getKnot(2).matrix();
    J_wrt_knots->block<3, 3>(0, 18) = J_wrt_knots->block<3, 3>(0, 18).eval() * so3_spline.getKnot(3).matrix();
    
    J_wrt_knots->block<3, 3>(3, 0) = J_wrt_knots->block<3, 3>(3, 0).eval() * so3_spline.getKnot(0).matrix();
    J_wrt_knots->block<3, 3>(3, 6) = J_wrt_knots->block<3, 3>(3, 6).eval() * so3_spline.getKnot(1).matrix();
    J_wrt_knots->block<3, 3>(3, 12) = J_wrt_knots->block<3, 3>(3, 12).eval() * so3_spline.getKnot(2).matrix();
    J_wrt_knots->block<3, 3>(3, 18) = J_wrt_knots->block<3, 3>(3, 18).eval() * so3_spline.getKnot(3).matrix();

    return true;
}

bool IMUUpdater::UpdateUsingIMUVector(const std::vector<TGK::BaseType::IMUDataConstPtr>& imu_vec,
                                      CameraFrame* cam1, CameraFrame* cam2, CameraFrame* cam3, CameraFrame* cam4,
                                      State* state) {
    const double dt1 = cam2->timestamp - cam1->timestamp;
    const double dt2 = cam3->timestamp - cam2->timestamp;
    const double dt3 = cam4->timestamp - cam3->timestamp;
    const double mean_dt = (dt1 + dt2 + dt3) / 3.;

    constexpr double kMaxTimeDiff = 0.001;
    if (std::abs(dt1 - mean_dt) > kMaxTimeDiff ||
        std::abs(dt2 - mean_dt) > kMaxTimeDiff ||
        std::abs(dt3 - mean_dt) > kMaxTimeDiff) {
        LOG(ERROR) << "[UpdateUsingIMUVector]: Time diff is too large.";
        return false;
    }

    // First create the B-Spline.
    basalt::So3Spline<4, double> so3_spline(dt1 * kSecToNanoSec, cam1->timestamp * kSecToNanoSec);
    basalt::RdSpline<3, 4, double> rd_spline(dt1 * kSecToNanoSec, cam1->timestamp * kSecToNanoSec);
    
    // Convert Camera pose to IMU pose.
    Eigen::Matrix3d G_R_I1 = cam1->G_R_C * C_R_I_;
    Eigen::Vector3d G_p_I1 = cam1->G_p_C + cam1->G_R_C * C_p_I_;

    Eigen::Matrix3d G_R_I2 = cam2->G_R_C * C_R_I_;
    Eigen::Vector3d G_p_I2 = cam2->G_p_C + cam2->G_R_C * C_p_I_;

    Eigen::Matrix3d G_R_I3 = cam3->G_R_C * C_R_I_;
    Eigen::Vector3d G_p_I3 = cam3->G_p_C + cam3->G_R_C * C_p_I_;

    Eigen::Matrix3d G_R_I4 = cam4->G_R_C * C_R_I_;
    Eigen::Vector3d G_p_I4 = cam4->G_p_C + cam4->G_R_C * C_p_I_;

    ForceOrthogonal(&G_R_I1);
    ForceOrthogonal(&G_R_I2);
    ForceOrthogonal(&G_R_I3);
    ForceOrthogonal(&G_R_I4);

    // Spline knots.
    so3_spline.knots_push_back(Sophus::SO3<double>(G_R_I1));
    so3_spline.knots_push_back(Sophus::SO3<double>(G_R_I2));
    so3_spline.knots_push_back(Sophus::SO3<double>(G_R_I3));
    so3_spline.knots_push_back(Sophus::SO3<double>(G_R_I4));
    rd_spline.knots_push_back(G_p_I1);
    rd_spline.knots_push_back(G_p_I2);
    rd_spline.knots_push_back(G_p_I3);
    rd_spline.knots_push_back(G_p_I4);


    // Evaluate jacobian for each imu data.
    std::vector<Eigen::Matrix<double, 6, 1>> res_vec;
    std::vector<Eigen::Matrix<double, 6, 6>> J_wrt_bias_vec;
    std::vector<Eigen::Matrix<double, 6, 24>> J_wrt_knots_vec;
    for (const auto& imu_data : imu_vec) {
        Eigen::Matrix<double, 6, 1> one_residual;
        Eigen::Matrix<double, 6, 6> one_J_wrt_bias;
        Eigen::Matrix<double, 6, 24> one_J_wrt_knots;
        if (!EvaluateOneJacobian(so3_spline, rd_spline, state->gyro_bias.bg, state->acc_bias.ba, *imu_data, 
                            &one_residual, &one_J_wrt_bias, &one_J_wrt_knots)) {
            LOG(WARNING) << "[UpdateUsingIMUVector]: Faild to evaluate one imu jacobian.";
            continue;
        }

        res_vec.push_back(one_residual);
        J_wrt_bias_vec.push_back(one_J_wrt_bias);
        J_wrt_knots_vec.push_back(one_J_wrt_knots);
    }

    /// Stack to big Jacobain.
    const size_t num_rows = 6 * res_vec.size();
    Eigen::VectorXd res(num_rows);
    Eigen::MatrixXd J_wrt_bias(num_rows, 6);
    Eigen::MatrixXd J_wrt_knots(num_rows, 24);

    for (size_t i = 0; i < res_vec.size(); ++i) {
        size_t row = i * 6;
        res.block<6, 1>(row, 0) = res_vec[i];
        J_wrt_bias.block<6, 6>(row, 0) = J_wrt_bias_vec[i];
        J_wrt_knots.block<6, 24>(row, 0) = J_wrt_knots_vec[i];
    }

    /// Compute Ji_wrt_Jc.
    Eigen::Matrix<double, 24, 24> Hic;
    Hic.setZero();
    int base_id = 0;
    Hic.block<3, 3>(base_id, base_id) = C_R_I_.transpose();
    Hic.block<3, 3>(base_id + 3, 0) = -cam1->G_R_C * TGK::Util::Skew(C_p_I_);
    Hic.block<3, 3>(base_id + 3, base_id + 3) = Eigen::Matrix3d::Identity();
    base_id += 6;
    Hic.block<3, 3>(base_id, base_id) = C_R_I_.transpose();
    Hic.block<3, 3>(base_id + 3, 0) = -cam2->G_R_C * TGK::Util::Skew(C_p_I_);
    Hic.block<3, 3>(base_id + 3, base_id + 3) = Eigen::Matrix3d::Identity();
    base_id += 6;
    Hic.block<3, 3>(base_id, base_id) = C_R_I_.transpose();
    Hic.block<3, 3>(base_id + 3, 0) = -cam3->G_R_C * TGK::Util::Skew(C_p_I_);
    Hic.block<3, 3>(base_id + 3, base_id + 3) = Eigen::Matrix3d::Identity();
    base_id += 6;
    Hic.block<3, 3>(base_id, base_id) = C_R_I_.transpose();
    Hic.block<3, 3>(base_id + 3, 0) = -cam4->G_R_C * TGK::Util::Skew(C_p_I_);
    Hic.block<3, 3>(base_id + 3, base_id + 3) = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd Hcs(num_rows, 24);
    Hcs = J_wrt_knots * Hic;

    /// Build the big covariance.
    LOG(ERROR) << "State_size: " << state->covariance.rows() << ", cams: " << state->camera_frames.size();
    Eigen::MatrixXd Hx(num_rows, state->covariance.rows());
    Hx.setZero();
    Hx.block(0, state->gyro_bias.state_idx, num_rows, 6) = J_wrt_bias;
    Hx.block(0, cam1->state_idx, num_rows, 24) = Hcs;

    Eigen::MatrixXd noise_V(num_rows, num_rows);
    noise_V.setZero();
    for (size_t i = 0; i < res_vec.size(); ++i) {
        int idx = i * 6;
        noise_V.block<3, 3>(idx, idx) = Eigen::Matrix3d::Identity() * config_.gyro_noise;
        noise_V.block<3, 3>(idx+3, idx+3) = Eigen::Matrix3d::Identity() * config_.acc_noise;
    }

    /// EKF update.
    EKFUpdate(Hx, res, noise_V, state);

    static std::fstream file;
    if (!file.is_open()) {
        file.open("/home/yds/data.csv", std::ios::out);
    }
    file << "Bg: " 
         << std::fixed << state->gyro_bias.bg[0] << ", "
         << std::fixed << state->gyro_bias.bg[1] << ", "
         << std::fixed << state->gyro_bias.bg[2] << ", Ba, "
         << std::fixed << state->acc_bias.ba[0] << ", "
         << std::fixed << state->acc_bias.ba[1] << ", "
         << std::fixed << state->acc_bias.ba[2] << "\n";
    file.flush();

    return true;
}

}  // namespace FilterFusion