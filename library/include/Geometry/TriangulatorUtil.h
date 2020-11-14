#pragma once 

#include <vector>
#include <Eigen/Core>

namespace TGK {
namespace Geometry {

bool TriangulateDLT(const std::vector<Eigen::Matrix3d>& C_R_Gs, 
                    const std::vector<Eigen::Vector3d>& C_p_Gs,
                    const std::vector<Eigen::Vector2d>& NSP_points,
                    Eigen::Vector3d* G_p);

bool RefineGlobalPoint(const std::vector<Eigen::Matrix3d>& C_R_Gs, 
                       const std::vector<Eigen::Vector3d>& C_p_Gs,
                       const std::vector<Eigen::Vector2d>& im_points,
                       Eigen::Vector3d* G_p);

}  // namespace Geometry
}  // namespace TGK