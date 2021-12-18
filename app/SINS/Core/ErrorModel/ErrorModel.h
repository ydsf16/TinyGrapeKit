#pragma once
#pragma once

#include <Eigen/Eigen>
#include "Index.h"
#include "InsState.h"
#include "SensorDataTypes.h"

namespace SINS {

class ErrorModel {
public:
    static KFMat ComputeFMatrix(const InsState &ins_state, const ImuData &imu_data);
    static KFMat ComputePhiMatrix(const InsState &ins_state, const ImuData &imu_data);
    static KFMat ComputeGMatrix(const InsState &ins_state);
};

} // namespace SINS