#pragma once

#include <Eigen/Core>
#include <vector>

#include <opencv2/opencv.hpp>

namespace TGK {
namespace ImageProcessor {

class FeatureTracker {
public:
    struct Config {
        int max_num_corners = 200;
        double quality_level = 0.01;
        double min_dist = 30.;
    };

    FeatureTracker(const Config& config);

    void TrackImage(const cv::Mat& image, 
                    std::vector<Eigen::Vector2d>* tracked_pts, 
                    std::vector<long int>* tracked_pt_ids,
                    std::vector<long int>* lost_pt_ids = nullptr,
                    std::set<long int>* new_pt_ids = nullptr);

private:
    void CreateMask(const int width, const int heigth, const std::vector<Eigen::Vector2d>& points, cv::Mat* mask);

    Config config_;
    static long int corner_id_;

    cv::Mat last_image_;
    std::vector<cv::Point2f> last_cv_pts_;
    std::vector<long int> last_pt_ids_;
};

}  // namespace ImageProcessor
} // namespace TGK