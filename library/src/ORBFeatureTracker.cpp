#include <ImageProcessor/ORBFeatureTracker.h>

#include <glog/logging.h>

namespace TGK {
namespace ImageProcessor {

ORBFeatureTracker::ORBFeatureTracker(const Config& config) : config_(config) { 
    orb_extractor_ = std::make_shared<ORB_SLAM2::ORBextractor>(
        config_.num_fts, config_.scale_factor, config_.num_levels, config_.init_th_fast, config_.min_th_fast);
}

int DescriptorDistance (const cv::Mat &a, const cv::Mat &b) {
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for ( int i=0; i<8; i++, pa++, pb++ ) {
        unsigned  int v = *pa ^ *pb;
        v = v - ( ( v >> 1 ) & 0x55555555 );
        v = ( v & 0x33333333 ) + ( ( v >> 2 ) & 0x33333333 );
        dist += ( ( ( v + ( v >> 4 ) ) & 0xF0F0F0F ) * 0x1010101 ) >> 24;
    }

    return dist;
}

void ORBFeatureTracker::TrackImage(const cv::Mat& image, 
                                   std::vector<Eigen::Vector2d>* tracked_pts, 
                                   std::vector<long int>* tracked_pt_ids,
                                   std::vector<long int>* lost_pt_ids,
                                   std::set<long int>* new_pt_ids) {
    // First detect orb features on the current frame.
    std::vector<cv::KeyPoint> cur_keypoints;
    cv::Mat cur_des;
    (*orb_extractor_)(image, cv::Mat(), cur_keypoints, cur_des);

    // Match with the current frame.
    std::vector <bool> cur_match_flag(cur_keypoints.size(), false);
    std::vector <bool> last_match_flag(last_keypoints_.size(), false);
    std::vector<long int> cur_pt_ids(cur_keypoints.size(), -1);
    for (size_t i = 0; i < last_keypoints_.size(); ++i) {
        const cv::KeyPoint& last_kp = last_keypoints_[i];
        const cv::Mat& last_kp_des = last_des_.row(i);
        const long int last_id = last_pt_ids_[i];

        int min_dist = std::numeric_limits<int>::max();
        int min_idx = -1;
        int second_min_dist = std::numeric_limits<int>::max();
        int second_min_idx = -1;
        for (size_t j = 0; j < cur_keypoints.size(); ++j) {
            if (cur_match_flag[j]) { continue; }
            const cv::Mat& cur_kp_des = cur_des.row(j);

            // Compute distance
            const int dist = DescriptorDistance(last_kp_des, cur_kp_des);

            if (dist < min_dist) {
                second_min_dist = min_dist;
                second_min_idx = min_idx;
                
                min_dist = dist;
                min_idx = j;
            } else if (dist < second_min_dist) {
                second_min_dist = dist;
                second_min_idx = j;
            }
        }

        // Matched featrues.
        if (min_idx >= 0 && second_min_idx >= 0 &&
            config_.first_two_dist_ratio * min_dist <= second_min_dist &&
            min_dist < 20) {
            cur_match_flag[min_idx] = true;
            last_match_flag[i] = true;
            cur_pt_ids[min_idx] = last_id;

            // Collect matcher
            const cv::KeyPoint& match_kp = cur_keypoints[min_idx];
            tracked_pt_ids->push_back(last_id);
            tracked_pts->emplace_back(match_kp.pt.x, match_kp.pt.y);
        } else { // Lost features.
            lost_pt_ids->push_back(last_id);
        }
    }
    // TODO: F RANSAC rejection.
    

    // Collect new features;
    for (size_t i = 0; i < cur_keypoints.size(); ++i) {
        if (cur_match_flag[i]) { continue; }
        const cv::KeyPoint& cur_kp = cur_keypoints[i];
        
        const long int id = corner_id_;
        ++corner_id_;
        new_pt_ids->insert(id);
        cur_pt_ids[i] = id;

        // tracked_pts->emplace_back(cur_kp.pt.x, cur_kp.pt.y);
        // tracked_pt_ids->push_back(id);
    }

    last_keypoints_ = cur_keypoints;
    last_des_ = cur_des;
    last_pt_ids_ = cur_pt_ids;
}

void ORBFeatureTracker::DeleteFeature(const long int pt_id) {
    const auto iter = std::find(last_pt_ids_.begin(), last_pt_ids_.end(), pt_id);
    if (iter == last_pt_ids_.end()) { return; }
    int idx = std::distance(last_pt_ids_.begin(), iter);
    last_pt_ids_.erase(iter);
    for (size_t i = idx; i < last_pt_ids_.size(); ++i) {
        last_des_.row(i+1).copyTo(last_des_.row(i));
    }
}

}  // namespace ImageProcessor
} // namespace TGK