#pragma once

#include <memory>

#include <Eigen/Eigen>

namespace SINS {

class EarthModel {
public:
    struct EarthParam {
        // Default: WGS84.
        double f = 1.0 / 298.257223563; // 扁率.
        double Re = 6378137;            // 长半轴
        double wie = 7.2921151467e-5;   // Earth rotation rate [rad/s]
    };

public:
    EarthModel() : EarthModel(EarthParam()) { }
    explicit EarthModel(const EarthParam &param);

    // Singleton.
    static EarthModel *Instance() {
        static EarthModel earth_mode;
        return &earth_mode;
    }
    
    void SetEarthParam(const EarthParam &param) {
        param_ = param;
    }

    void GetRmRn(double latitude, double *Rm, double *Rn);
    Eigen::Vector3d GetGravity(double latitude, double hight = 0);
    Eigen::Vector3d GetWie(double latitude);
    static Eigen::Vector3d GetWen(double east_vel, double north_vel, double latitude, double height, double Rm, double Rn);

private:
    EarthParam param_;
};

} // namespace SINS