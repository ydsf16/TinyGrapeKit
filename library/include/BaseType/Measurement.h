#pragma once

#include <memory>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

namespace TGK {
namespace BaseType {

enum class MeasureType {
    kUnknown,
    kWheel,
    kMonoImage
};

struct Measurement {
    double timestamp;
    MeasureType type;
};

struct WheelData : Measurement {
    WheelData() { 
        type = MeasureType::kWheel;
    }
    
    double left;
    double right;
};
using WheelDataConstPtr = std::shared_ptr<const WheelData>;
using WheelDataPtr = std::shared_ptr<WheelData>;

struct MonoImageData : Measurement {
    MonoImageData() {
        type = MeasureType::kMonoImage;
    }

    cv::Mat image;
};
using MonoImageDataConstPtr = std::shared_ptr<const MonoImageData>;
using MonoImageDataPtr = std::shared_ptr<MonoImageData>;

struct SimMonoImageData : MonoImageData {
    std::vector<Eigen::Vector2d> features;
    std::vector<long int> feature_ids;
};
using SimMonoImageDataConstPtr = std::shared_ptr<const SimMonoImageData>;
using SimMonoImageDataPtr = std::shared_ptr<SimMonoImageData>;

}  // namespace BaseType
}  // namespace TGK