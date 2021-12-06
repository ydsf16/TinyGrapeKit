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

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
int main(int argc, char **argv) {
    const std::string imu_path = argv[1];
    const std::string res_save_path = argv[2];

    auto imu_vec = ParseImuFile(imu_path);
    std::cout << "Imu size: " << imu_vec.size() << std::endl;

    // 1. Initialization.
    size_t number_init_imus = 25000;
    Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_gyro = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < number_init_imus; ++i) {
        auto imu = imu_vec[i];
        mean_acc += imu->acc;
        mean_gyro += imu->gyro;
    }
    mean_acc  /=  number_init_imus;
    mean_gyro /= number_init_imus;

    Eigen::Matrix3d C_nb = SINS::StationaryCoarseAlign(mean_acc, mean_gyro); // 

    size_t kFirstIdx = 70000;
    InsState ins_state;
    ins_state.time = imu_vec[kFirstIdx]->time;
    ins_state.lon_lat_hei << 116.252733994444 * kDegToRad, 40.091633386111 * kDegToRad, 0.0;
    ins_state.velocity.setZero();
    ins_state.orientation = C_nb;
    ins_state.acc_bias.setZero();
    ins_state.gyro_bias.setZero();

    Eigen::Matrix3d C_nb_fix = ( Eigen::AngleAxisd(116.2591 * kDegToRad, Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(1.1720 * kDegToRad, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(-0.6693 * kDegToRad, Eigen::Vector3d::UnitX()) ).toRotationMatrix();
    ins_state.orientation = C_nb_fix;
    InsUpdater::UpdateEarthParams(&ins_state);
    
    // 2. Update.
    std::ofstream save_file(res_save_path, std::ios::out);
    for (size_t i = kFirstIdx+1; i < imu_vec.size(); ++i) {
        InsUpdater::UpdateInsState(imu_vec[i-1], imu_vec[i], &ins_state);

        Eigen::Vector3d yaw_pitch_roll = RotMatToEuler(ins_state.orientation) * kRadToDeg;

        save_file << std::fixed << ins_state.time << ", " << std::setprecision(12)
            << ins_state.lon_lat_hei.x() * kRadToDeg << ", " 
            << ins_state.lon_lat_hei.y() * kRadToDeg << ", " 
            << ins_state.lon_lat_hei.z() << ", " 
            << ins_state.velocity.x() << ", " 
            << ins_state.velocity.y() << ", " 
            << ins_state.velocity.z() << ", "
            << yaw_pitch_roll.x() << ", "
            << yaw_pitch_roll.y() << ", "
            << yaw_pitch_roll.z() << std::endl;
    }

    save_file.close();

    return EXIT_SUCCESS;
}