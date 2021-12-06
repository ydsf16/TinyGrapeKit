#include "InsUpdate/InsUpdater.h" 
#include "Utils/SO3.h"

#include <iostream>

namespace SINS {

bool InsUpdater::UpdateInsState(double delta_t,
                                const Eigen::Vector3d &delta_theta1, const Eigen::Vector3d &delta_theta2, 
                                const Eigen::Vector3d &delta_v1, const Eigen::Vector3d &delta_v2,
                                InsState *ins_state) {
    // Compute earth params.
    UpdateEarthParams(ins_state);

    // Copy last state.
    InsState last_ins_state = *ins_state;

    std::cout << "Gravity: " << ins_state->gravity.transpose() << std::endl;
    std::cout << "Rm: " << ins_state->Rm << ", Rn: " << ins_state->Rn << std::endl;

    std::cout << "delta_v1: " << std::fixed << (delta_v1 + delta_v2).transpose() / delta_t << std::endl;
    
    bool res_vel = UpdateVelocity(delta_t, delta_theta1, delta_theta2, delta_v1, delta_v2, ins_state);
    bool res_ori = UpdateOrientation(delta_t, delta_theta1, delta_theta2, ins_state);
    bool res_pos = UpdatePosition(delta_t, last_ins_state.velocity, ins_state);

    // std::cin.ignore();

    return res_vel && res_ori && res_pos;
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

    std::cout << "AVG ACC: " << std::fixed << 0.5 * (begin_acc + end_acc).transpose() << std::endl;

    ins_state->time = end_time;
    return UpdateInsState(delta_t, delta_theta1, delta_theta2, delta_v1, delta_v2, ins_state);                     
}

bool InsUpdater::UpdateInsState(const ImuData::ConstPtr begin_imu, const ImuData::ConstPtr end_imu, InsState *ins_state) {
    return UpdateInsState(begin_imu->time, begin_imu->acc - ins_state->acc_bias, begin_imu->gyro - ins_state->gyro_bias, 
                          end_imu->time, end_imu->acc - ins_state->acc_bias, end_imu->gyro - ins_state->gyro_bias,
                          ins_state);
}

Eigen::Vector3d InsUpdater::GetWen(double v_E, double v_N, double latitude, double height, double Rm, double Rn) {
    return Eigen::Vector3d(
        -v_N / (Rm + height),
        v_E / (Rn + height),
        v_N * std::tan(latitude) / (Rn + height)
    );
}
Eigen::Vector3d InsUpdater::GetWen(const InsState &ins_state) {
    return GetWen(ins_state.velocity.x(), ins_state.velocity.y(), 
                  ins_state.lon_lat_hei.y(), ins_state.lon_lat_hei.z(), 
                  ins_state.Rm, ins_state.Rn);
}

void InsUpdater::UpdateEarthParams(InsState *ins_state) {
    ins_state->gravity = EarthModel::Instance()->GetGravity(ins_state->lon_lat_hei[1], ins_state->lon_lat_hei[2]);
    EarthModel::Instance()->GetRmRn(ins_state->lon_lat_hei[1], &ins_state->Rm, &ins_state->Rn);
}

/*** Single Update ***/
bool InsUpdater::UpdateOrientation(double delta_t,
                                   const Eigen::Vector3d &delta_theta1, const Eigen::Vector3d &delta_theta2,
                                   InsState *ins_state) {
    const Eigen::Vector3d w_in = EarthModel::Instance()->GetWie(ins_state->lon_lat_hei[1]); //  + GetWen(*ins_state); TODO: Hard code.
    const Eigen::Vector3d phi_nn = -w_in * delta_t;
    const Eigen::Matrix3d C_nn = SO3Exp(phi_nn);

    const Eigen::Vector3d phi_bb = delta_theta1 + delta_theta2 + (2.0/3.0) * delta_theta1.cross(delta_theta2);
    const Eigen::Matrix3d C_bb = SO3Exp(phi_bb);

    ins_state->orientation = C_nn * ins_state->orientation.eval() * C_bb;

    return true;
}

bool InsUpdater::UpdateVelocity(double delta_t, 
                                const Eigen::Vector3d &delta_theta1, const Eigen::Vector3d &delta_theta2, 
                                const Eigen::Vector3d &delta_v1, const Eigen::Vector3d &delta_v2,
                                InsState *ins_state) {
    // Rot
    const Eigen::Vector3d delta_theta = delta_theta1 + delta_theta2;
    const Eigen::Vector3d delta_v = delta_v1 + delta_v2;
    const Eigen::Vector3d delta_v_rot = 0.5 * delta_theta.cross(delta_v);

    // Scul
    const Eigen::Vector3d delta_v_scul = ( 2.0 / 3.0) * (delta_theta1.cross(delta_v2) + delta_v1.cross(delta_theta2));

    const Eigen::Vector3d w_in = EarthModel::Instance()->GetWie(ins_state->lon_lat_hei[1]) + GetWen(*ins_state);
    
    // Vsf.
    const Eigen::Vector3d delta_v_sf = (Eigen::Matrix3d::Identity() - 0.5 * delta_t *  SkewMat(w_in)) * 
        ins_state->orientation.transpose() * (delta_v + delta_v_rot + delta_v_scul);
    
    const Eigen::Vector3d delta_v_corgm = 
        (-(2.0 * EarthModel::Instance()->GetWie(ins_state->lon_lat_hei[1]) + GetWen(*ins_state)).cross(ins_state->velocity) + ins_state->gravity) * delta_t;
    
    ins_state->velocity = ins_state->velocity.eval() + delta_v_corgm + delta_v_sf;

    //ins_state->velocity = ins_state->velocity + ins_state->orientation.transpose() * delta_v + ins_state->gravity * delta_t;
    
    //std::cout << "delta vel corgm: " << std::fixed << delta_v_corgm.transpose() << ", delta_v_sf: " << delta_v_sf.transpose() << ", delta_v: " << (delta_v_sf + delta_v_corgm).transpose() << std::endl;

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