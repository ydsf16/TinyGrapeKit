#pragma once

#include <memory>

#include <ImageProcessor/FeatureTracker.h>
#include <ImageProcessor/ORBextractor.h>

namespace TGK {
namespace ImageProcessor {

class ORBFeatureTracker : public FeatureTracker {
public:
    struct Config {
        int num_fts = 1000;
        float scale_factor = 1.2;
        int num_levels = 8;
        int init_th_fast = 20;
        int min_th_fast = 10;

        double first_two_dist_ratio = 3.;
    };

    ORBFeatureTracker(const Config& Config);

    virtual void TrackImage(const cv::Mat& image, 
                            std::vector<Eigen::Vector2d>* tracked_pts, 
                            std::vector<long int>* tracked_pt_ids,
                            std::vector<long int>* lost_pt_ids = nullptr,
                            std::set<long int>* new_pt_ids = nullptr);

    virtual void DeleteFeature(const long int pt_id);

private:
    const Config config_;

    std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor_;
    
    std::vector<cv::KeyPoint> last_keypoints_;
    cv::Mat last_des_;
    std::vector<long int> last_pt_ids_;
};

}  // namespace ImageProcessor
}  // namespace TGK