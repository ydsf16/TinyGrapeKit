#include <iostream>
#include <fstream>
#include <iomanip>

#include "InsUpdate/InsUpdater.h"
#include "Alignment/StationaryCoarseAlignment.h"
#include "Utils/SO3.h"

using namespace SINS;

ImuData::Ptr ParseImu(const std::string &line_str) {
    ImuData::Ptr imu_data = std::make_shared<ImuData>();

    // Split line.
    std::vector<std::string> line_data_vec;
    std::stringstream ss(line_str);
    std::string value_str;
    while (std::getline(ss, value_str, ',')) { line_data_vec.push_back(value_str); }
    if (line_data_vec.size() != 7) {
        return nullptr;
    }

    imu_data->time = std::stod(line_data_vec[6]);
    imu_data->gyro << std::stod(line_data_vec[0]), std::stod(line_data_vec[1]), std::stod(line_data_vec[2]);
    imu_data->acc << std::stod(line_data_vec[3]), std::stod(line_data_vec[4]), std::stod(line_data_vec[5]);

    return imu_data;
}

std::vector<ImuData::Ptr> ParseImuFile(const std::string &imu_path) {
    std::vector<ImuData::Ptr> imu_datas;

    std::fstream imu_file(imu_path, std::ios::in);
    std::string line_str;
    std::vector<std::string> line_data_vec;
    while (std::getline(imu_file, line_str)) {
        auto imu_ptr = ParseImu(line_str);

        if (imu_ptr != nullptr) {
            imu_datas.push_back(imu_ptr);
        }
    }  

    return imu_datas;
}

class StreamToFile : public std::fstream {
public:
    StreamToFile(const std::string &file_path) : std::fstream(), file_path_(file_path) { 
        open(file_path_, std::ios::out | std::ios::app);
    }

    ~StreamToFile() {
        flush();
        close();
    }
private:
    std::string file_path_;
};

int main(int argc, char **argv) {
    const std::string imu_path = argv[1];
    const std::string res_save_path = argv[2];

    auto imu_vec = ParseImuFile(imu_path);
    std::cout << "Imu size: " << imu_vec.size() << std::endl;

    // Set initial state.
    size_t kFirstIdx = 70000;
    InsState ins_state;
    ins_state.time = imu_vec[kFirstIdx]->time;
    ins_state.lon_lat_hei << 116.252733994444 * kDegToRad, 40.091633386111 * kDegToRad, 40.0;
    ins_state.velocity.setZero();
    ins_state.orientation = AttToMat(Eigen::Vector3d(-0.6693, 1.1720, 116.2591) * kDegToRad); 
    ins_state.acc_bias.setZero();
    ins_state.gyro_bias.setZero();
    InsUpdater::UpdateEarthParams(&ins_state);
    
    // 2. Update.
    std::ofstream save_file(res_save_path, std::ios::out);
    for (size_t i = kFirstIdx; i < imu_vec.size(); i += 2) {
        // Update.
        auto imu1 = imu_vec[i];
        auto imu2 = imu_vec[i+1];
        double time = imu2->time;
        InsUpdater::UpdateInsState(0.02, imu1->gyro, imu2->gyro, imu1->acc, imu2->acc, &ins_state);
        ins_state.time = time;

        // Quaterniond to euler angle.
        Eigen::Vector3d att = MatToAtt(ins_state.orientation.toRotationMatrix());

        save_file << std::fixed << std::setprecision(12)
            << att.x() << ", "
            << att.y() << ", "
            << att.z() << ", "
            << ins_state.velocity.x() << ", " 
            << ins_state.velocity.y() << ", " 
            << ins_state.velocity.z() << ", "
            << ins_state.lon_lat_hei.x() << ", " 
            << ins_state.lon_lat_hei.y() << ", " 
            << ins_state.lon_lat_hei.z() << ", "
            << std::to_string(ins_state.time) << std::endl; 
    }

    save_file.close();

    return EXIT_SUCCESS;
}
