#pragma once

#include <Eigen/Core>

#include <VWO/Visualizer.h>

namespace VWO {

struct CamParam {
    double fx;
    double fy;
    double cx;
    double cy;
    double k1;
    double k2;
    double p1;
    double p2;
};

struct WheelParam {
    double kl;
    double kr;
    double b;
};

struct ExtrinsicParam {
    Eigen::Matrix3d O_R_C;
    Eigen::Vector3d O_p_C;
};

struct Parameter {
    // Camera instrinc.
    CamParam cam_intrinsic;
    // Wheel Intrinsic.
    WheelParam wheel_param;
    // Extrinsic.
    ExtrinsicParam extrinsic;

    Visualizer::Config viz_config;
};

}  // namespace VWO