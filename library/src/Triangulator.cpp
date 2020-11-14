#include <Geometry/Triangulator.h>
#include <Geometry/TriangulatorUtil.h>

#include <Eigen/Dense>
#include <glog/logging.h>

namespace TGK {
namespace Geometry {

Triangulator::Triangulator(const Camera::CameraPtr camera) : camera_(camera) { } 

bool Triangulator::Triangulate(const std::vector<Eigen::Matrix3d>& G_R_Cs, 
                               const std::vector<Eigen::Vector3d>& G_p_Cs,
                               const std::vector<Eigen::Vector2d>& im_pts,
                               Eigen::Vector3d* G_p) {
    // Convert camera pose.
    std::vector<Eigen::Matrix3d> C_R_Gs;
    std::vector<Eigen::Vector3d> C_p_Gs;
    C_R_Gs.reserve(G_R_Cs.size());
    C_p_Gs.reserve(G_p_Cs.size());
    for (size_t i = 0; i < G_R_Cs.size(); ++i) {
        const Eigen::Matrix3d& G_R_C = G_R_Cs[i];
        const Eigen::Vector3d& G_p_C = G_p_Cs[i];

        const Eigen::Matrix3d C_R_G = G_R_C.transpose();
        const Eigen::Vector3d C_p_G = -C_R_G * G_p_C;

        C_R_Gs.push_back(C_R_G);
        C_p_Gs.push_back(C_p_G);
    }

    // Project image points to NSP.
    std::vector<Eigen::Vector2d> NSP_pts;
    NSP_pts.reserve(im_pts.size());
    for (size_t i = 0; i < im_pts.size(); ++i) {
        const Eigen::Vector2d& im_pt = im_pts[i];
        const Eigen::Vector3d NSP_pt = camera_->ImageToCamera(im_pt, 1.);   
        NSP_pts.push_back(NSP_pt.head<2>());
    }

    // Linear triangulation.
    if (!TriangulateDLT(C_R_Gs, C_p_Gs, NSP_pts, G_p)) {
        LOG(WARNING) << "[Triangulate]: Failed to triangulate point using the DLT method.";
        return false;
    }

    // Non-linear refinement.
    

    return true;
}

}  // namespace Geometry
}  // namespace TGK