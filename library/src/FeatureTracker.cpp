#include <ImageProcessor/FeatureTracker.h>

#include <glog/logging.h>

namespace TGK {
namespace ImageProcessor {

long int FeatureTracker::corner_id_ = 0;

FeatureTracker::FeatureTracker(const Config& config) : config_(config) { }

void FeatureTracker::TrackImage(const cv::Mat& image, 
                                std::vector<Eigen::Vector2d>* tracked_pts, 
                                std::vector<long int>* tracked_pt_ids,
                                std::vector<long int>* lost_pt_ids,
                                std::set<long int>* new_pt_ids) {
    
    tracked_pts->clear();
    tracked_pt_ids->clear();
    if (lost_pt_ids != nullptr) { lost_pt_ids->clear(); }
    if (new_pt_ids != nullptr) { new_pt_ids->clear(); }

    // 1. Track features from the last frame.
    if (!last_image_.empty() && !last_pt_ids_.empty()) {
        std::vector<cv::Point2f> track_cv_pts;
        std::vector<uchar> track_status;
        std::vector<float> error;
        // Forward track.
        cv::calcOpticalFlowPyrLK(last_image_, image, last_cv_pts_, track_cv_pts, track_status, error);
        
        // Backward track.
        std::vector<cv::Point2f> bacK_track_cv_pts;
        std::vector<uchar> back_track_status;
        std::vector<float> back_error;
        cv::calcOpticalFlowPyrLK(image, last_image_, track_cv_pts, bacK_track_cv_pts, back_track_status, back_error);

        for (size_t i = 0; i < track_cv_pts.size(); ++i) {
            const auto& pt_id = last_pt_ids_[i];

            if (track_status[i] == 0 || back_track_status[i] == 0) { 
                if (lost_pt_ids != nullptr) { lost_pt_ids->push_back(pt_id); }
                continue; 
            }
            const double dx = track_cv_pts[i].x - bacK_track_cv_pts[i].x;
            const double dy = track_cv_pts[i].y - bacK_track_cv_pts[i].y;
            const double dist = std::sqrt(dx * dx + dy * dy);
            if (dist > 5.) { 
                if (lost_pt_ids != nullptr) { lost_pt_ids->push_back(pt_id); }
                continue; 
            }

            tracked_pts->emplace_back(track_cv_pts[i].x, track_cv_pts[i].y);
            tracked_pt_ids->push_back(pt_id);
        }
    }

    // 2. Create new features.
    const int num_new_pts = config_.max_num_corners - tracked_pt_ids->size();
    if (num_new_pts > 0) {
        cv::Mat mask;
        CreateMask(image.cols, image.rows, *tracked_pts, &mask);

        std::vector<cv::Point2f> new_pts;
        cv::goodFeaturesToTrack(image, new_pts, num_new_pts, config_.quality_level, config_.min_dist, mask);

        for (const auto& pt : new_pts) {
            const auto id = corner_id_;
            ++corner_id_;

            if (new_pt_ids != nullptr) { new_pt_ids->insert(id); }

            tracked_pts->emplace_back(pt.x, pt.y);
            tracked_pt_ids->push_back(id);
        }   
    }

    // 3. Reset last frame data.
    last_image_ = image;
    last_pt_ids_ = *tracked_pt_ids;
    last_cv_pts_.clear();
    last_cv_pts_.reserve(tracked_pts->size());
    for (size_t i = 0; i < tracked_pts->size(); ++i) {
        last_cv_pts_.emplace_back(tracked_pts->at(i)[0], tracked_pts->at(i)[1]);
    }
}

void FeatureTracker::CreateMask(const int width, const int heigth, 
                                const std::vector<Eigen::Vector2d>& points, cv::Mat* mask) {
    *mask = cv::Mat(heigth, width, CV_8UC1, cv::Scalar(1));
    for (const auto& pt : points) {
        cv::circle(*mask, cv::Point2i(pt[0], pt[1]), config_.min_dist, 0, -1);
    }
} 

}  // namespace ImageProcessor
}  // namespace TGK