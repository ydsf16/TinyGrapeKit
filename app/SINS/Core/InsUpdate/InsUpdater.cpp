#include "InsUpdate/InsUpdater.h" 
#include "Utils/SO3.h"

#include <iostream>

namespace SINS {

bool InsUpdater::UpdateInsState(double delta_t,
                                const Eigen::Vector3d &delta_theta1, const Eigen::Vector3d &delta_theta2, 
                                const Eigen::Vector3d &delta_v1, const Eigen::Vector3d &delta_v2,
                                InsState *ins_state) {
    // Compute earth params.
    if (ins_state->update_earth) {
        UpdateEarthParams(ins_state);
    } 

    // Copy last state.
    InsState last_ins_state = *ins_state;

    bool res_vel = UpdateVelocity(delta_t, delta_theta1, delta_theta2, delta_v1, delta_v2, ins_state);
    bool res_ori = UpdateOrientation(delta_t, delta_theta1, delta_theta2, ins_state);
    bool res_pos = UpdatePosition(delta_t, last_ins_state.velocity, ins_state);

    // std::cin.ignore();

    return true;
}

bool InsUpdater::UpdateInsState(double begin_time, const Eigen::Vector3d &begin_acc, const Eigen::Vector3d &begin_gyro,
                                double end_time, const Eigen::Vector3d &end_acc, const Eigen::Vector3d &end_gyro,
                                InsState *ins_state) {
    double delta_t = end_time - begin_time;
    const Eigen::Vector3d mid_acc = begin_acc + (end_acc - begin_acc) * 0.5;
    const Eigen::Vector3d mid_gyro = begin_gyro + (end_gyro - begin_gyro) * 0.5;
    const Eigen::Vector3d delta_theta1 = 0.25 * delta_t * (begin_gyro + mid_gyro);
    const Eigen::Vector3d delta_theta2 = 0.25 * delta_t * (mid_gyro + end_gyro);
    const Eigen::Vector3d delta_v1 = 0.25 * delta_t * (begin_acc + mid_acc);
    const Eigen::Vector3d delta_v2 = 0.25 * delta_t * (mid_acc + end_acc);

    ins_state->time = end_time;
    return UpdateInsState(delta_t, delta_theta1, delta_theta2, delta_v1, delta_v2, ins_state);                     
}

bool InsUpdater::UpdateInsState(const ImuData::ConstPtr begin_imu, const ImuData::ConstPtr end_imu, InsState *ins_state) {
    return UpdateInsState(begin_imu->time, begin_imu->acc - ins_state->acc_bias, begin_imu->gyro - ins_state->gyro_bias, 
                          end_imu->time, end_imu->acc - ins_state->acc_bias, end_imu->gyro - ins_state->gyro_bias,
                          ins_state);
}

void InsUpdater::UpdateEarthParams(InsState *ins_state) {
    UpdateEarthParams(ins_state->lon_lat_hei[1], ins_state->lon_lat_hei[2], ins_state->velocity[0], ins_state->velocity[1], ins_state);
}

void InsUpdater::UpdateEarthParams(double latitude, double height, double east_vel, double north_vel,
                                   InsState *ins_state) {
    ins_state->gravity = EarthModel::Instance()->GetGravity(latitude, height);
    ins_state->wie = EarthModel::Instance()->GetWie(latitude);
    EarthModel::Instance()->GetRmRn(latitude, &ins_state->Rm, &ins_state->Rn);
    ins_state->wen = EarthModel::GetWen(east_vel, north_vel, latitude, height, ins_state->Rm, ins_state->Rn);
    ins_state->win = ins_state->wie + ins_state->wen;
}

/*** Single Update ***/
bool InsUpdater::UpdateOrientation(double delta_t,
                                   const Eigen::Vector3d &delta_theta1, const Eigen::Vector3d &delta_theta2,
                                   InsState *ins_state) {
    const Eigen::Vector3d phi_nn = -ins_state->win * delta_t;
    Eigen::Matrix3d C_nn = SO3Exp(phi_nn);

    const Eigen::Vector3d phi_bb = delta_theta1 + delta_theta2 + (2.0/3.0) * delta_theta1.cross(delta_theta2);
    const Eigen::Matrix3d C_bb = SO3Exp(phi_bb);

    ins_state->orientation = C_nn * ins_state->orientation.eval() * C_bb;
    ins_state->orientation = NormalizeRotMat(ins_state->orientation);

    return true;
}

bool InsUpdater::UpdateVelocity(double delta_t, 
                                const Eigen::Vector3d &delta_theta1, const Eigen::Vector3d &delta_theta2, 
                                const Eigen::Vector3d &delta_v1, const Eigen::Vector3d &delta_v2,
                                InsState *ins_state) {
    const Eigen::Vector3d delta_theta = delta_theta1 + delta_theta2;
    const Eigen::Vector3d delta_v = delta_v1 + delta_v2;
    const Eigen::Vector3d delta_v_rot = 0.5 * delta_theta.cross(delta_v);

    // Scul
    const Eigen::Vector3d delta_v_scul = ( 2.0 / 3.0) * (delta_theta1.cross(delta_v2) + delta_v1.cross(delta_theta2));

    // Vsf.
    const Eigen::Vector3d delta_v_sf = ins_state->orientation * (delta_v + delta_v_rot + delta_v_scul);
    
    const Eigen::Vector3d delta_v_corgm = 
         (-(2.0 * ins_state->wie + ins_state->wen).cross(ins_state->velocity) + ins_state->gravity) * delta_t;
    
    // ins_state->velocity = ins_state->velocity.eval() + delta_v_corgm + delta_v_sf;

    ins_state->velocity = ins_state->velocity + Eigen::Quaterniond(ins_state->orientation) * delta_v + ins_state->gravity * delta_t;

    return true;
}

bool InsUpdater::UpdatePosition(double delta_t, const Eigen::Vector3d &vel_m_1, InsState *ins_state) {
    Eigen::Matrix3d M;
    M << 0.0, 1.0 / (ins_state->Rm + ins_state->lon_lat_hei[2]), 0.0,
         1.0 / ((ins_state->Rn + ins_state->lon_lat_hei[2]) * std::cos(ins_state->lon_lat_hei[1])), 0.0, 0.0,
         0.0, 0.0, 1.0;

    ins_state->lon_lat_hei = ins_state->lon_lat_hei + M * (vel_m_1 + ins_state->velocity) * delta_t * 0.5;

    return true;
}

}  // namespace SINS