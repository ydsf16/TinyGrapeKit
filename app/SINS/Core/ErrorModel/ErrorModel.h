#pragma once

#include <Eigen/Eigen>

#include "Base/Index.h"
#include "Base/InsState.h"

namespace SINS {

class ErrorModel {
public:
    static KFMat ComputeFMatrix(const InsState &ins_state);
    static KFMat ComputePhiMatrix(const InsState &ins_state);
    static Eigen::Matrix<double, kKFStateDim, 6> ComputeGMatrix(const InsState &ins_state);
};

} // namespace SINS