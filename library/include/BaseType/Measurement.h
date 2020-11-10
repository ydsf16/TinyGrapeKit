#pragma once

#include <opencv2/opencv.hpp>

namespace TGK {
namespace BaseType {

enum class MeasureType {
    kUnknown,
    kWheelDist,
    kMonoImage
};

struct Measurement {
    double timestamp;
    MeasureType type;
};

struct WheelDist : Measurement {
    WheelDist() { 
        type = MeasureType::kWheelDist;
    }
    
    double left;
    double right;
};

struct MonoImage : Measurement {
    MonoImage() {
        type = MeasureType::kMonoImage;
    }

    cv::Mat image;
};

}  // namespace BaseType
}  // namespace TGK