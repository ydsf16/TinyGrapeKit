#include <Geometry/TriangulatorUtil.h>

#include <Eigen/Dense>
#include <glog/logging.h>

namespace TGK {
namespace Geometry {

bool TriangulateDLT(const std::vector<Eigen::Matrix3d>& C_R_Gs, 
                    const std::vector<Eigen::Vector3d>& C_p_Gs,
                    const std::vector<Eigen::Vector2d>& NSP_points,
                    Eigen::Vector3d* G_p) {
    const size_t n_obs = C_R_Gs.size();
    Eigen::MatrixXd A(2 * n_obs, 4);
    Eigen::Matrix<double, 1, 4> P1, P2, P3;
    size_t idx = 0;
    for (size_t i = 0; i < n_obs; ++i) {
        const auto& pt = NSP_points[i];
        const double x = pt[0];
        const double y = pt[1];

        const auto&  R = C_R_Gs[i];
        const auto&  p = C_p_Gs[i];
        P1 << R.block<1, 3>(0, 0), p(0);
        P2 << R.block<1, 3>(1, 0), p(1);
        P3 << R.block<1, 3>(2, 0), p(2);
        
        A.block<1, 4>(idx, 0) = x * P3 - P1;
        ++idx;
        A.block<1, 4>(idx, 0) = y * P3 - P2;
        ++idx;
    }

    // Solve Ax = 0.
    const Eigen::Matrix4d H = A.transpose() * A;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen_solver(H);
    if (eigen_solver.info() != Eigen::Success) {
        LOG(WARNING) << "[TriangulateDLT]: Failed to compute eigenvector of H.";
        return false;
    }
    const Eigen::Vector4d X = eigen_solver.eigenvectors().leftCols<1>();

    if (std::abs(X[3]) < 1e-12) {
        LOG(WARNING) << "[TriangulateDLT]: X[3] is too small!";
        return false;
    }
    *G_p = X.topRows<3>() / X[3];

    return true;                     
}

bool RefineGlobalPoint(const std::vector<Eigen::Matrix3d>& C_R_Gs, 
                       const std::vector<Eigen::Vector3d>& C_p_Gs,
                       const std::vector<Eigen::Vector2d>& im_points,
                       Eigen::Vector3d* G_p) {
                               
}

}  // namespace Geometry
}  // namespace TGK