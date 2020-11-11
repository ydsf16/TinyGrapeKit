#pragma once

#include <deque>
#include <mutex>
#include <vector>

#include <Eigen/Core>

namespace VWO {

class Visualizer {
public:
    struct Config {
        double cam_size = 0.08;
        double cam_line_width = 3.;
        double view_point_x = 0.;
        double view_point_y = -0.7;
        double view_point_z = -1.8;
        double view_point_f = 500.;

        int max_traj_length = 100000;
        int max_num_features = 5000;
    };

    Visualizer(const Config& config);

    void DrawCameras(const std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>& camera_poses);

    void DrawWheelPose(const Eigen::Matrix3d& G_R_O, const Eigen::Vector3d& G_p_O);

    void DrawFeatures(const std::vector<Eigen::Vector3d>& features);

private:
    void Run();

    const Config config_;
    
    // Data buffer.
    std::mutex data_buffer_mutex_;
    std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> camera_poses_;
    std::deque<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> wheel_traj_;
    std::deque<Eigen::Vector3d> features_;
};

}  // namespace VWO