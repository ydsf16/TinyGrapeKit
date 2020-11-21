#pragma once

#include <memory>
#include <vector>
#include <set>


#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include <ImageProcessor/FeatureTracker.h>

namespace TGK {
namespace ImageProcessor {

class OpenVinsTracker : public FeatureTracker {
public:
    struct Config {
        int num_pts = 200;
        int fast_th = 10;
        int grid_x = 10;
        int grid_y = 5;
        int max_px_dist = 9;

        bool use_klt = false;
    };

    OpenVinsTracker(const Config& config, const Eigen::Matrix<double, 8, 1>& cam_intrin);

    virtual void TrackImage(const cv::Mat& image, 
                            std::vector<Eigen::Vector2d>* tracked_pts, 
                            std::vector<long int>* tracked_pt_ids,
                            std::vector<long int>* lost_pt_ids = nullptr,
                            std::set<long int>* new_pt_ids = nullptr);

    virtual void DeleteFeature(const long int pt_id);


private:
    Config config_;
};

}  // namespace ImageProcessor
}  // namespace TGK