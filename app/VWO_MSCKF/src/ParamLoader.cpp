#include <VWO/ParamLoader.h>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>

namespace VWO {

void LoadParam(const std::string& param_file, Parameter* params) {
    cv::FileStorage cv_params(param_file, cv::FileStorage::READ);
    
    // Load camera param.
    params->cam_intrinsic.fx = cv_params["Camera.fx"];
    params->cam_intrinsic.fy = cv_params["Camera.fy"];
    params->cam_intrinsic.cx = cv_params["Camera.cx"];
    params->cam_intrinsic.cy = cv_params["Camera.cy"];
    params->cam_intrinsic.s  = cv_params["Camera.s"];

    params->cam_intrinsic.k1 = cv_params["Camera.k1"];
    params->cam_intrinsic.k2 = cv_params["Camera.k2"];
    params->cam_intrinsic.p1 = cv_params["Camera.p1"];
    params->cam_intrinsic.p2 = cv_params["Camera.p2"];
    params->cam_intrinsic.k3 = cv_params["Camera.k3"];

    // Load wheel intrinsic.
    params->wheel_param.kl = cv_params["Wheel.kl"];
    params->wheel_param.kr = cv_params["Wheel.kr"];
    params->wheel_param.b = cv_params["Wheel.b"];
    params->wheel_param.noise_factor = cv_params["Wheel.noise_factor"];

    // Load extrinsic.
    cv::Mat cv_O_R_C;
    cv_params["Extrinsic.O_R_C"] >> cv_O_R_C;
    cv::Mat cv_O_p_C;
    cv_params["Extrinsic.O_p_C"] >> cv_O_p_C;
    params->extrinsic.O_R_C << 
        cv_O_R_C.at<double>(0, 0), cv_O_R_C.at<double>(0, 1), cv_O_R_C.at<double>(0, 2), 
        cv_O_R_C.at<double>(1, 0), cv_O_R_C.at<double>(1, 1), cv_O_R_C.at<double>(1, 2), 
        cv_O_R_C.at<double>(2, 0), cv_O_R_C.at<double>(2, 1), cv_O_R_C.at<double>(2, 2), 
    params->extrinsic.O_p_C << cv_O_p_C.at<double>(0, 0), cv_O_p_C.at<double>(1, 0), cv_O_p_C.at<double>(2, 0);
}

}  // namespace VWO