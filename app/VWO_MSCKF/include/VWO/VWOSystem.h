#pragma once 

#include <memory>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include <TGK/Camera/Camera.h>
#include <TGK/DataSynchronizer/WheelImageSynchronizer.h>

#include <VWO/Initializer.h>
#include <VWO/Parameter.h>
#include <VWO/Propagator.h>
#include <VWO/State.h>
#include <VWO/Visualizer.h>

namespace VWO {

class VWOSystem {
public:
    VWOSystem(const std::string& param_file);

    bool FeedWheelData(const double timestamp, const double left, const double right);

    bool FeedImageData(const double timestamp, const cv::Mat& image);

    bool GetCameraPoses(std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>* cam_poses);

    bool GetWheelPose(Eigen::Matrix3d* G_R_O, Eigen::Vector3d* G_p_O);

private:
    std::shared_ptr<TGK::Camera::Camera> camera_;
    std::unique_ptr<TGK::DataSynchronizer::WheelImageSynchronizer> data_sync_;
    std::unique_ptr<Initializer> initializer_;
    std::unique_ptr<Visualizer> viz_;
    std::unique_ptr<Propagator> propagator_;

    bool initialized_;
    Parameter param_;

    State state_;
};

}  // namespace VWO