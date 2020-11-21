#include <ImageProcessor/OpenVinsTracker.h>

#include <glog/logging.h>

#include <map>

// OpenVINS header.
#include <track/TrackKLT.h>
#include <track/TrackDescriptor.h>

namespace TGK {
namespace ImageProcessor {

std::shared_ptr<ov_core::TrackBase> gkOvKLTTracker = nullptr;
double gkImageTimestamp = 0.;
constexpr size_t kCamId = 0;

OpenVinsTracker::OpenVinsTracker(const Config& config, const Eigen::Matrix<double, 8, 1>& cam_intrin) 
    : config_(config) {
    
    if (config.use_klt) {
        gkOvKLTTracker = std::make_shared<ov_core::TrackKLT>(
            config_.num_pts, 0, config_.fast_th, config_.grid_x, config_.grid_y,config_.max_px_dist);
    } else {
        gkOvKLTTracker = std::make_shared<ov_core::TrackDescriptor>(
            config_.num_pts, 0, config_.fast_th, config_.grid_x, config_.grid_y,config_.max_px_dist);
    }

    std::map<size_t, Eigen::VectorXd> camera_calib;
    camera_calib[kCamId] = cam_intrin;
    std::map<size_t, bool> camera_fisheye;
    camera_fisheye[kCamId] = false;
    gkOvKLTTracker->set_calibration(camera_calib, camera_fisheye);
}

void OpenVinsTracker::TrackImage(const cv::Mat& image, 
                                    std::vector<Eigen::Vector2d>* tracked_pts, 
                                    std::vector<long int>* tracked_pt_ids,
                                    std::vector<long int>* lost_pt_ids,
                                    std::set<long int>* new_pt_ids)  {
    tracked_pts->clear();
    tracked_pt_ids->clear();
    
    gkImageTimestamp += 0.01;
    cv::Mat tmp_img = image;
    gkOvKLTTracker->feed_monocular(gkImageTimestamp, tmp_img, kCamId);

    // Get lost features.
    if (lost_pt_ids != nullptr) { 
        std::vector<ov_core::Feature*> feats_lost =
            gkOvKLTTracker->get_feature_database()->features_not_containing_newer(gkImageTimestamp);

        for (const auto ft : feats_lost) {
            lost_pt_ids->push_back(ft->featid);
        }
    }

    const std::vector<ov_core::Feature*> feature_tracked = 
        gkOvKLTTracker->get_feature_database()->features_containing(gkImageTimestamp);
    for (const auto ft : feature_tracked) {
        tracked_pt_ids->push_back(ft->featid);

        const std::vector<Eigen::VectorXf>& uvs = ft->uvs.at(kCamId);
        const std::vector<double>& times = ft->timestamps.at(kCamId);
        
        const Eigen::VectorXf& uv = uvs.back();
        tracked_pts->emplace_back(uv[0], uv[1]);
    }
}

void OpenVinsTracker::DeleteFeature(const long int pt_id) {
    ov_core::Feature* ft = gkOvKLTTracker->get_feature_database()->get_feature(pt_id);
    if (ft == nullptr) { return; }

    // TODO: Releae memory?
    ft->to_delete = true;
    gkOvKLTTracker->get_feature_database()->cleanup();
}

}  // namespace ImageProcessor
}  // namespace TGK