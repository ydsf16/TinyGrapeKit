#pragma once

#include "Earth/EarthModel.h"
#include "InsState.h"
#include "Base/SensorDataTypes.h"

namespace SINS {

class InsUpdater {
public:
    /******* Main Update Interface ********/
    static void UpdateInsState(double time,
                               const Eigen::Vector3d &delta_theta1, const Eigen::Vector3d &delta_theta2, 
                               const Eigen::Vector3d &delta_v1, const Eigen::Vector3d &delta_v2,
                               InsState *ins_state,
                               bool fix_height = false);

    static void UpdateInsState(double begin_time, const Eigen::Vector3d &begin_acc, const Eigen::Vector3d &begin_gyro,
                               double end_time, const Eigen::Vector3d &end_acc, const Eigen::Vector3d &end_gyro,
                               InsState *ins_state);

    static void UpdateInsState(const ImuData::ConstPtr begin_imu, const ImuData::ConstPtr end_imu, InsState *ins_state);

    static void UpdateEarthParams(InsState *ins_state);

    static void UpdateEarthParams(double latitude, double height, double east_vel, double north_vel, InsState *ins_state);

private:
    /*** Single Update ***/
    static void UpdateOrientation(double delta_t,
                                  const Eigen::Vector3d &delta_theta1, const Eigen::Vector3d &delta_theta2,
                                  InsState *ins_state);

    static void UpdateVelocity(double delta_t, 
                               const Eigen::Vector3d &delta_theta1, const Eigen::Vector3d &delta_theta2, 
                               const Eigen::Vector3d &delta_v1, const Eigen::Vector3d &delta_v2,
                               InsState *ins_state);

    static void UpdatePosition(double delta_t, const Eigen::Vector3d &vel_m_1, InsState *ins_state);
};

}  // namespace SINS