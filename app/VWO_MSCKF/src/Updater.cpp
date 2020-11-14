#include <VWO/Updater.h>

#include <unordered_set>

namespace VWO {

Updater::Updater(const std::shared_ptr<TGK::ImageProcessor::FeatureTracker> feature_tracker)
    : feature_tracker_(feature_tracker) { }

void Updater::UpdateState(const cv::Mat& image, const bool marg_oldest, State* state) {
    // Track image.
    std::vector<Eigen::Vector2d> tracked_pts; 
    std::vector<long int> tracked_pt_ids;
    std::vector<long int> lost_pt_ids;
    std::set<long int> new_pt_ids;
    feature_tracker_->TrackImage(image, &tracked_pts, &tracked_pt_ids, &lost_pt_ids, &new_pt_ids);

    // Assuming a new clone has been inserted into the sliding window.
    CameraFramePtr new_cam_frame = state->camera_frames.back();

    // Insert tracked features to frame.
    for (size_t i = 0; i < tracked_pt_ids.size(); ++i) {
        const long int ft_id = tracked_pt_ids[i];
        const Eigen::Vector2d& pt = tracked_pts[i];
        // Insert feature to camera frame.
        new_cam_frame->id_pt_map[ft_id] = pt;
    }

    ///  Collect features use to update.
    // 1. Tracked lost features.
    std::unordered_set<long int> lost_ft_ids_set(lost_pt_ids.begin(), lost_pt_ids.end());
    // 2. We always marginalize the last camera frame. So we need to collect all features in the last frame.
    if (marg_oldest) {
        const CameraFramePtr marg_cam = state->camera_frames.front();
        for (const auto& id_pt : marg_cam->id_pt_map) {
            lost_ft_ids_set.insert(id_pt.first);
        }
    }

    // Collect image point & camera state pairs.
    std::vector<std::vector<std::pair<Eigen::Vector2d, CameraFramePtr>>> point_cam_pairs;
    for (const long int id : lost_ft_ids_set) {
        std::vector<std::pair<Eigen::Vector2d, CameraFramePtr>> one_feature;
        for (const auto cam_fm : state->camera_frames) {
            const auto iter = cam_fm->id_pt_map.find(id);
            if (iter == cam_fm->id_pt_map.end()) { continue; }
            const Eigen::Vector2d& im_pt = iter->second;
            one_feature.emplace_back(im_pt, cam_fm);
        }
    }

    /// Triangulate points.
    // img point, map point, camera state.
    std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector3d, CameraFramePtr>>> observations;
    


}

}  // namespace VWO