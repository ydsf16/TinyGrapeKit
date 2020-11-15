#pragma once

#include <memory>

#include <opencv2/opencv.hpp>

#include <TGK/Geometry/Triangulator.h>
#include <TGK/ImageProcessor/FeatureTracker.h>
#include <VWO/State.h>

namespace VWO {

class Updater {
public:
    struct Config {
        double param = 0.;
    };

    Updater(const std::shared_ptr<TGK::ImageProcessor::FeatureTracker> feature_tracker,
            const std::shared_ptr<TGK::Geometry::Triangulator> triangulator);

    void UpdateState(const cv::Mat& image, const bool marg_oldest, State* state, 
                     std::vector<Eigen::Vector2d>* all_features,
                     std::vector<Eigen::Vector2d>* new_features,
                     std::vector<Eigen::Vector3d>* map_points);

private:
    const std::shared_ptr<TGK::ImageProcessor::FeatureTracker> feature_tracker_; 
    const std::shared_ptr<TGK::Geometry::Triangulator> triangulator_;
};

}  // namespace VWO