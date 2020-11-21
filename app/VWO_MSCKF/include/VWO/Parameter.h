#pragma once

#include <Eigen/Core>

#include <VWO/Visualizer.h>
#include <VWO/Updater.h>
#include <TGK/Geometry/Triangulator.h>
#include <TGK/ImageProcessor/OpenVinsTracker.h>

namespace VWO {

struct CamParam {
    double fx;
    double s;
    double fy;
    double cx;
    double cy;
    double k1;
    double k2;
    double p1;
    double p2;
    double k3;

    int width;
    int height;
};

struct WheelParam {
    double kl;
    double kr;
    double b;
    
    double noise_factor;
};

struct ExtrinsicParam {
    Eigen::Matrix3d O_R_C;
    Eigen::Vector3d O_p_C;
};

struct SysConfig {
    int sliding_window_size;
    bool compute_raw_odom;
};

struct Parameter {
    // Camera instrinc.
    CamParam cam_intrinsic;
    // Wheel Intrinsic.
    WheelParam wheel_param;
    // Extrinsic.
    ExtrinsicParam extrinsic;

    Visualizer::Config viz_config;
    TGK::Geometry::Triangulator::Config tri_config;
    TGK::ImageProcessor::OpenVinsTracker::Config tracker_config;
    Updater::Config updater_config;

    SysConfig sys_config;
};

}  // namespace VWO