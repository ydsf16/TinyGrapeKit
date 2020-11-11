#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

namespace VWO {

class Visualizer {
public:
    struct Config {
        double cam_size = 2.;
        double cam_line_width = 3.;
        double point_size = 2.;
        double wheel_frame_size = 2.;
        double view_point_x = 0.;
        double view_point_y = 0.;
        double view_point_z = 200.;
        double view_point_f = 500.;

        double img_heigh = 140;
        double img_width = 320;

        int max_traj_length = 100000;
        int max_num_features = 5000;
    };

    Visualizer(const Config& config);

    void DrawCameras(const std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>& camera_poses);

    void DrawWheelPose(const Eigen::Matrix3d& G_R_O, const Eigen::Vector3d& G_p_O);

    void DrawFeatures(const std::vector<Eigen::Vector3d>& features);

    void DrawImage(const cv::Mat& image);

private:
    void Run();

    void DrawOneCamera(const Eigen::Matrix3d& G_R_C, const Eigen::Vector3d& G_p_C);
    void DrawCameras();
    void DrawTraj();
    void DrawFeatures();
    void DrawWheeFrame(const Eigen::Matrix3d& G_R_O, const Eigen::Vector3d& G_p_O);
    void DrawWheeFrame();

    const Config config_;
    
    // Thread.
    std::shared_ptr<std::thread> viz_thread_;

    // Data buffer.
    std::mutex data_buffer_mutex_;
    std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> camera_poses_;
    std::deque<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> wheel_traj_;
    std::deque<Eigen::Vector3d> features_;

    cv::Mat image_;
};

}  // namespace VWO