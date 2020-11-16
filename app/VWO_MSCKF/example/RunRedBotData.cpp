
#include <fstream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>

#include <VWO/Visualizer.h>
#include <VWO/VWOSystem.h>

int main(int argc, char** argv) {
    if (argc != 3) {
        LOG(ERROR) << "Please input: Rosbag file!";
        return EXIT_FAILURE;
    }

    // Config glog.
    FLAGS_alsologtostderr  = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_minloglevel      = 0;

    /****** Load Configs ******/
    const std::string param_file  = argv[1];
    const std::string rosbag_path = argv[2];
    const std::string kImageTopic = "/kinect2/qhd/image_color";
    const std::string kWheelTopic = "/rbot/encoder";


    // Create VWO system.
    VWO::VWOSystem vwo_sys(param_file);

    /****** Play Data ******/
    rosbag::Bag bag;
    bag.open(rosbag_path);  // BagMode is Read by default
    for (rosbag::MessageInstance const m : rosbag::View(bag)) {
        if (m.getTopic() == kWheelTopic) {
            const auto& msg = m.instantiate<geometry_msgs::QuaternionStamped>();

            // Feed wheel data to system.
            const double left_enc_cnt = 0.5 * (msg->quaternion.x + msg->quaternion.y);
            const double right_enc_cnt = 0.5 * (msg->quaternion.z + msg->quaternion.w);

            LOG(INFO) << "LEFT RIGH: " << std::fixed << msg->header.stamp.toSec() << ", " << left_enc_cnt << ", " << right_enc_cnt;
            vwo_sys.FeedWheelData(msg->header.stamp.toSec(), left_enc_cnt, right_enc_cnt);
        } else if (m.getTopic() == kImageTopic) {
            const auto& msg = m.instantiate<sensor_msgs::Image>();
            const auto& img_ptr = cv_bridge::toCvShare(msg);

            // Feed to system.
            cv::Mat gray_img;
            cv::cvtColor(img_ptr->image, gray_img, CV_BGR2GRAY);
            vwo_sys.FeedImageData(msg->header.stamp.toSec(), gray_img);

            LOG(INFO) << "Image: " << std::fixed << msg->header.stamp.toSec();
        }
    }

    bag.close();

    return EXIT_SUCCESS;
}