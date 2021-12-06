#include "Earth/EarthModel.h"

namespace SINS {

EarthModel::EarthModel(const EarthParam &param) : param_(param) { }

void EarthModel::GetRmRn(double latitude, double *Rm, double *Rn) {
    double sin_lat = std::sin(latitude);
    double sin_lat2 = sin_lat * sin_lat;
    double one_minus = 1.0 - (2.0 - param_.f) * param_.f * sin_lat2;
    double sqrt_one_minus = std::sqrt(one_minus);

    *Rm = param_.Re * (1.0 - param_.f) * (1.0 - param_.f) / (one_minus * sqrt_one_minus);
    *Rn = param_.Re / sqrt_one_minus;
}

Eigen::Vector3d EarthModel::GetGravity(double latitude, double height) {
    // NOTE: CGC200. Not WGS84.
    double sin_lat = std::sin(latitude);
    double sin_2lat = std::sin(2.0 * latitude);
    double g0 = 9.7803253361 * (1.0 + 5.30244e-3 * sin_lat * sin_lat - 5.82e-6 * sin_2lat * sin_2lat);
    double g = g0 / ((1.0 + height / param_.Re) * (1.0 + height / param_.Re));
    
    constexpr double kBeta3 = 8.08e-9;
   // return Eigen::Vector3d(0.0, -kBeta3 * height * sin_2lat, -g);

    return Eigen::Vector3d(0.0, 0.0, -g0);
}

Eigen::Vector3d EarthModel::GetWie(double latitude) {
    return Eigen::Vector3d(0.0, param_.wie * std::cos(latitude), param_.wie * std::sin(latitude));
}

Eigen::Vector3d EarthModel::GetWen(double east_vel, double north_vel, double latitude, double height, double Rm, double Rn) {
    return Eigen::Vector3d(
        -north_vel / (Rm + height),
        east_vel / (Rn + height),
        north_vel * std::tan(latitude) / (Rn + height)
    );
}

}  // namespace SINS