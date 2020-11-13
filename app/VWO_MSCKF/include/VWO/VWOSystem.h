#pragma once 

#include <memory>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include <TGK/Camera/Camera.h>
#include <TGK/DataSynchronizer/WheelImageSynchronizer.h>

#include <VWO/Initializer.h>
#include <VWO/Parameter.h>
#include <VWO/State.h>

namespace VWO {

class VWOSystem {
public:
    VWOSystem(const std::string& param_file);

    bool FeedIMU(const double timestamp, const double left, const double right);

    bool FeedImage(const double timestamp, const cv::Mat& image);

    bool GetCameraPoses(std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>* cam_poses);

    bool GetWheelPose(Eigen::Matrix3d* G_R_O, Eigen::Vector3d* G_p_O);

private:
    std::shared_ptr<TGK::Camera::Camera> camera_;
    std::unique_ptr<TGK::DataSynchronizer::WheelImageSynchronizer> data_sync_;
    std::unique_ptr<Initializer> initializer_;

    bool initialized_;
    Parameter param_;

    State state_;
};

}  // namespace VWO