#pragma once

#include <memory>

#include <opencv2/opencv.hpp>
#include <VWO/State.h>
#include <TGK/ImageProcessor/FeatureTracker.h>

namespace VWO {

class Updater {
public:
    struct Config {
        double param = 0.;
    };

    Updater(const std::shared_ptr<TGK::ImageProcessor::FeatureTracker> feature_tracker);

    void UpdateState(const cv::Mat& image, const bool marg_oldest, State* state);

private:
    const std::shared_ptr<TGK::ImageProcessor::FeatureTracker> feature_tracker_; 
};

}  // namespace VWO