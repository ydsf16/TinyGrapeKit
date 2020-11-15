#include <VWO/UpdaterUtil.h>

#include <Eigen/Dense>

#include <TGK/Util/Util.h>

namespace VWO {

void LeftNullspaceProjection(const Eigen::MatrixXd& Hx, 
                             const Eigen::MatrixXd& Hf, 
                             const Eigen::VectorXd& res, 
                             Eigen::MatrixXd* H,
                             Eigen::VectorXd* r) {
    const size_t rows = Hf.rows();
    const Eigen::HouseholderQR<Eigen::MatrixXd> qr(Hf);
    const Eigen::MatrixXd Q = qr.householderQ();
    const Eigen::MatrixXd Q2_trans = Q.rightCols(rows - 3).transpose();

    *H = Q2_trans * Hx;
    *r = Q2_trans * res;
}

void CompressMeasurement(const Eigen::MatrixXd& H, 
                         const Eigen::VectorXd& r, 
                         Eigen::MatrixXd* H_cmp, 
                         Eigen::VectorXd* r_cmp) {
    if (H.rows() <= H.cols()) {
        *H_cmp = H;
        *r_cmp = r;
        return;
    }
    
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(H);
    Eigen::MatrixXd Q = qr.householderQ();
    Eigen::MatrixXd Q1_trans = Q.leftCols(H.cols()).transpose();

    *H_cmp = qr.matrixQR().topRows(H.cols()).triangularView<Eigen::Upper>();
    *r_cmp = Q1_trans * r;
}

void EKFUpdate(const Eigen::MatrixXd& H, 
               const Eigen::VectorXd& r, 
               const Eigen::MatrixXd& V,
               State* state) {
    const Eigen::MatrixXd P_minus = state->covariance;
    const Eigen::MatrixXd H_trans = H.transpose();
    const Eigen::MatrixXd S = H * P_minus * H_trans + V;
    const Eigen::MatrixXd S_inv = S.llt().solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));   
    const Eigen::MatrixXd K = P_minus * H_trans * S_inv;

    // Compute delta x.
    const Eigen::VectorXd delta_x = K * r;

    // Update.
    state->Update(delta_x);
    
    // Update covariance.
    const Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(state->covariance.rows(), state->covariance.rows()) - K * H;
    state->covariance = I_KH * P_minus * I_KH.transpose() + K * V * K.transpose();
}


void ComputePlaneConstraintResidualJacobian(const Eigen::Matrix3d& G_R_O, const Eigen::Vector3d& G_p_O,
                                            Eigen::Vector3d* res, Eigen::Matrix<double, 3, 6>* H) {
    const Eigen::Matrix3d pi_G_G = Eigen::Matrix3d::Identity();
    const double pi_z_G = 0.;
    
    Eigen::Matrix<double, 2, 3> Lambda;
    Lambda << 1., 0., 0.,
              0., 1., 0.;
    const Eigen::Vector3d e3 = Eigen::Vector3d::UnitZ();

    res->head<2>() = -Lambda * pi_G_G * G_R_O * e3;
    (*res)[2] = pi_z_G + e3.transpose() * pi_G_G * G_p_O;

    H->setZero();
    H->block<2, 3>(0, 0) = Lambda * pi_G_G * G_R_O * TGK::Util::Skew(e3);
    H->block<1, 3>(2, 3) = -e3.transpose() * pi_G_G;
}

}  // namespace VWO