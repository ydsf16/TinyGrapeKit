#include <FilterFusion/IMUUpdater.h>

#include <glog/logging.h>

#include <basalt/spline/so3_spline.h>
#include <basalt/spline/rd_spline.h>

namespace FilterFusion {

constexpr double kSecToNanoSec = 1e9;

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

    // Erase all imus before cam2.
    while (!imu_buffer_.empty() && imu_buffer_.front()->timestamp <= cam2->timestamp) {
        imu_buffer_.pop_front();
    }

    // Check if all data arrived between cam2 and cam3.
    if (imu_buffer_.empty() || 
        imu_buffer_.back()->timestamp < cam3->timestamp || 
        imu_buffer_.front()->timestamp >= cam3->timestamp) {
        LOG(INFO) << "[UpdateState]: Watting data.";
        return true;
    }

    // Collect all imu data in cam2 and cam3.
    std::vector<TGK::BaseType::IMUDataConstPtr> imu_vec;
    for (const auto imu_data : imu_buffer_) {
        if (imu_data->timestamp > cam3->timestamp) { break; }
        if (imu_data->timestamp >= cam2->timestamp && imu_data->timestamp <= cam3->timestamp) {
            imu_vec.push_back(imu_data);
        }
    }

    // Update using IMU.
    UpdateUsingIMUVector(imu_vec, &state->gyro_bias, &state->acc_bias, 
                         cam1.get(), cam2.get(), cam3.get(), cam4.get());

    // Remove all used ims.
    while (!imu_buffer_.empty() && imu_buffer_.front()->timestamp <= cam3->timestamp) {
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

    // Get acc.
    basalt::RdSpline<3, 4, double>::JacobianStruct rd_acc_Jacobian;
    basalt::RdSpline<3, 4, double>::JacobianStruct rd_Jacobian;
    const Eigen::Vector3d G_a_I = rd_spline.acceleration(timestamp, &rd_acc_Jacobian);
    const Eigen::Vector3d G_p_I = rd_spline.evaluate(timestamp, &rd_Jacobian);

    // Get omega.
    basalt::So3Spline<4, double>::JacobianStruct so3_vel_Jacobian;
    basalt::So3Spline<4, double>::JacobianStruct so3_Jacobian;
    const Eigen::Vector3d I_w_I = so3_spline.velocityBody(timestamp, &so3_vel_Jacobian);
    const Sophus::SO3<double> so3_G_R_I = so3_spline.evaluate(timestamp, &so3_Jacobian);
    const Eigen::Matrix3d G_R_I = so3_G_R_I.matrix();

    // Compute residual.
    residual->head<3>() = imu_data.gyro - (I_w_I + bg);
    residual->segment<3>(3) = imu_data.acc - (G_R_I.transpose() * (G_a_I + kGravity) + ba);
    
    // Compute Jacobians.
    J_wrt_bias->setZero();
    Eigen::Matrix<double, 6, 9> Ht;
    Ht.setZero();
    Ht.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    Ht.block<3, 3>(3, 0) = 


    return true;
}

bool IMUUpdater::UpdateUsingIMUVector(const std::vector<TGK::BaseType::IMUDataConstPtr>& imu_vec,
                                      GyroBias* gyro_bias, AccBias* acc_bias, 
                                      CameraFrame* cam1, CameraFrame* cam2, CameraFrame* cam3, CameraFrame* cam4) {
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
    basalt::So3Spline<4, double> so3_spline(mean_dt * kSecToNanoSec, cam1->timestamp * kSecToNanoSec);
    basalt::RdSpline<3, 4, double> rd_spline(mean_dt * kSecToNanoSec, cam1->timestamp * kSecToNanoSec);
    
    // Convert Camera pose to IMU pose.
    Eigen::Matrix3d G_R_I1 = cam1->G_R_C * C_R_I_;
    Eigen::Vector3d G_p_I1 = cam1->G_p_C + cam1->G_R_C * C_p_I_;

    Eigen::Matrix3d G_R_I2 = cam2->G_R_C * C_R_I_;
    Eigen::Vector3d G_p_I2 = cam2->G_p_C + cam2->G_R_C * C_p_I_;

    Eigen::Matrix3d G_R_I3 = cam3->G_R_C * C_R_I_;
    Eigen::Vector3d G_p_I3 = cam3->G_p_C + cam3->G_R_C * C_p_I_;

    Eigen::Matrix3d G_R_I4 = cam4->G_R_C * C_R_I_;
    Eigen::Vector3d G_p_I4 = cam4->G_p_C + cam4->G_R_C * C_p_I_;

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
    for (const auto& imu_data : imu_vec) {

    }

    return true;
}

}  // namespace FilterFusion