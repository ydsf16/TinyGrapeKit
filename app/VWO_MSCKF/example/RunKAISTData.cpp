#include <fstream>
#include <string>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>

bool LoadEncoderData(const std::string& encoder_file_path, std::unordered_map<std::string, std::string>* time_encoder_map) {
    std::ifstream encoder_file(encoder_file_path);
    if (!encoder_file.is_open()) {
        LOG(ERROR) << "[LoadEncoderData]: Failed to open encoder file.";
        return false;
    } 

    std::string line_str, time_str;
    while (std::getline(encoder_file, line_str)) {
        std::stringstream ss(line_str);
        if (!std::getline(ss, time_str, ',')) {
            LOG(ERROR) << "[LoadEncoderData]: Find a bad line in the encoder file.: " << line_str;
            return false;
        }

        time_encoder_map->emplace(time_str, line_str);
    }

    return true;
}

// 1. Config file.
// 2. Dataset folder.
int main(int argc, char** argv) {
    if (argc != 3) {
        LOG(ERROR) << "[main]: Please input config_file, data_folder.";
        return EXIT_FAILURE;
    }

    FLAGS_minloglevel = 0;
    FLAGS_colorlogtostderr = true;

    const std::string config_file = argv[1];
    const std::string data_folder = argv[2];

    // Load all encoder data to buffer.
    std::unordered_map<std::string, std::string> time_encoder_map;
    if (!LoadEncoderData(data_folder + "/sensor_data/encoder.csv", &time_encoder_map)) {
        LOG(ERROR) << "[main]: Failed to load encoder data.";
        return EXIT_FAILURE;
    } 

    std::ifstream file_data_stamp(data_folder + "/sensor_data/data_stamp.csv");
    if (!file_data_stamp.is_open()) {
        LOG(ERROR) << "[main]: Failed to open data_stamp file.";
        return EXIT_FAILURE;
    }

    std::vector<std::string> line_data_vec;
    line_data_vec.reserve(3);
    std::string line_str, value_str;
    while (std::getline(file_data_stamp, line_str)) {
        line_data_vec.clear();
        std::stringstream ss(line_str);
        while (std::getline(ss, value_str, ',')) { line_data_vec.push_back(value_str); }

        constexpr double kToSecond = 1e-9;
        const std::string& time_str = line_data_vec[0];
        const double timestamp = std::stod(time_str) * kToSecond;
        
        const std::string& sensor_type = line_data_vec[1];
        if (sensor_type == "stereo") {
            const std::string img_file = data_folder + "/image/stereo_left/" + time_str + ".png";
            const cv::Mat raw_image = cv::imread(img_file, CV_LOAD_IMAGE_ANYDEPTH);
            if (raw_image.empty()) {
                LOG(ERROR) << "[main]: Failed to open image at time: " << time_str;
                return EXIT_FAILURE;
            }
            
            // Convert raw image to gray image.
            cv::Mat gray_img;
            cv::cvtColor(raw_image, gray_img, cv::COLOR_BayerRG2GRAY);
            

        }

        if (sensor_type == "encoder") {
            if (time_encoder_map.find(time_str) == time_encoder_map.end()) {
                LOG(ERROR) << "[main]: Failed to find encoder data at time: " << time_str;
                return EXIT_FAILURE;
            }
            const std::string& encoder_str = time_encoder_map.at(time_str);
            std::stringstream enc_ss(encoder_str);
            line_data_vec.clear();
            while (std::getline(enc_ss, value_str, ',')) { line_data_vec.push_back(value_str); }

            const double left_enc_cnt = std::stod(line_data_vec[1]);
            const double right_enc_cnt = std::stod(line_data_vec[2]);
            
            // TODO: Send encoder data to system.
        }
    }

    return EXIT_SUCCESS;
}