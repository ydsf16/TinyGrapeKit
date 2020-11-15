#include <VWO/Updater.h>

#include <unordered_set>

namespace VWO {

Updater::Updater(const std::shared_ptr<TGK::ImageProcessor::FeatureTracker> feature_tracker,
                 const std::shared_ptr<TGK::Geometry::Triangulator> triangulator)
    : feature_tracker_(feature_tracker), triangulator_(triangulator) { }

void Updater::UpdateState(const cv::Mat& image, const bool marg_oldest, State* state, 
                          std::vector<Eigen::Vector2d>* tracked_features,
                          std::vector<Eigen::Vector2d>* new_features,
                          std::vector<Eigen::Vector3d>* map_points) {
    // Track image.
    std::vector<Eigen::Vector2d> tracked_pts; 
    std::vector<long int> tracked_pt_ids;
    std::vector<long int> lost_pt_ids;
    std::set<long int> new_pt_ids;
    feature_tracker_->TrackImage(image, &tracked_pts, &tracked_pt_ids, &lost_pt_ids, &new_pt_ids);

    // Collect features for observation.
    *tracked_features = tracked_pts; 
    for (size_t i = 0; i < tracked_pt_ids.size(); ++i) {
        const long long pt_id = tracked_pt_ids[i];
        if (new_pt_ids.count(pt_id) > 0) {
            new_features->push_back(tracked_pts[i]);
        }
    }

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
    std::vector<std::vector<std::pair<Eigen::Vector2d, CameraFramePtr>>> features_obs;
    for (const long int id : lost_ft_ids_set) {
        std::vector<std::pair<Eigen::Vector2d, CameraFramePtr>> one_feature;
        for (const auto cam_fm : state->camera_frames) {
            const auto iter = cam_fm->id_pt_map.find(id);
            if (iter == cam_fm->id_pt_map.end()) { continue; }
            const Eigen::Vector2d& im_pt = iter->second;
            one_feature.emplace_back(im_pt, cam_fm);
        }
        features_obs.push_back(one_feature);
    }

    struct FeatureObservation {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::vector<CameraFramePtr> camera_frame;
        std::vector<Eigen::Vector2d> im_pts;
        Eigen::Vector3d G_p;
    };

    /// Triangulate points.
    map_points->clear();
    map_points->reserve(lost_ft_ids_set.size());
    std::vector<FeatureObservation> features_full_obs;
    features_full_obs.reserve(features_obs.size());
    for (const std::vector<std::pair<Eigen::Vector2d, CameraFramePtr>>& one_ft_obs : features_obs) {
        FeatureObservation one_feaute;
        one_feaute.im_pts.reserve(one_ft_obs.size());
        one_feaute.camera_frame.reserve(one_ft_obs.size());
        std::vector<Eigen::Matrix3d> G_R_Cs;
        std::vector<Eigen::Vector3d> G_p_Cs;
        for (const auto& one_obs : one_ft_obs) {
            G_R_Cs.push_back(one_obs.second->G_R_C);
            G_p_Cs.push_back(one_obs.second->G_p_C);
            one_feaute.im_pts.push_back(one_obs.first);
            one_feaute.camera_frame.push_back(one_obs.second);
        }

        if (!triangulator_->Triangulate(G_R_Cs, G_p_Cs, one_feaute.im_pts, &one_feaute.G_p)) { continue; }
        features_full_obs.push_back(one_feaute);
        map_points->push_back(one_feaute.G_p);
    }

    
}

}  // namespace VWO