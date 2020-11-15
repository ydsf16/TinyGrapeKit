#pragma once

#include <Eigen/Core>

#include <VWO/State.h>

namespace VWO {

void LeftNullspaceProjection(const Eigen::MatrixXd& Hx, 
                             const Eigen::MatrixXd& Hf, 
                             const Eigen::VectorXd& res, 
                             Eigen::MatrixXd* H,
                             Eigen::VectorXd* r);

void CompressMeasurement(const Eigen::MatrixXd& H, 
                         const Eigen::VectorXd& r, 
                         Eigen::MatrixXd* H_cmp, 
                         Eigen::VectorXd* r_cmp);

void EKFUpdate(const Eigen::MatrixXd& H, 
               const Eigen::VectorXd& r, 
               const Eigen::MatrixXd& V,
               State* state);

void ComputePlaneConstraintResidualJacobian(const Eigen::Matrix3d& G_R_O, const Eigen::Vector3d& G_p_O,
                                            Eigen::Vector3d* res, Eigen::Matrix<double, 3, 6>* H);

} // namespace VWO