#pragma once 

#include <memory>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include <TGK/Camera/Camera.h>

namespace VWO {

class VWOSystem {
public:
    VWOSystem(const std::string& param_file);

    bool FeedIMU(const double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro);

    bool FeedImage(const double timestamp, const cv::Mat& image);

    bool GetCameraPoses(std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>* cam_poses);

    bool GetWheelPose(Eigen::Matrix3d* G_R_O, Eigen::Vector3d* G_p_O);

private:
    std::shared_ptr<TGK::Camera::Camera> camera_;
};

}  // namespace VWO