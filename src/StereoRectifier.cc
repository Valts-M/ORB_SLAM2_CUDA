#include "StereoRectifier.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace ORB_SLAM2 {


stereo_rectifier::stereo_rectifier(const std::string &strSettingPath, const bool fisheye = true){
    const YAML::Node params = YAML::LoadFile(strSettingPath);

    // set image size
    const cv::Size img_size(params["Camera.height"].as<int>(), params["Camera.width"].as<int>());
    // set camera matrices
    const auto K_l = parse_vector_as_mat(cv::Size(3, 3), params["StereoRectifier.K_left"].as<std::vector<double>>());
    const auto K_r = parse_vector_as_mat(cv::Size(3, 3), params["StereoRectifier.K_right"].as<std::vector<double>>());
    // set rotation matrices
    const auto R_l = parse_vector_as_mat(cv::Size(3, 3), params["StereoRectifier.R_left"].as<std::vector<double>>());
    const auto R_r = parse_vector_as_mat(cv::Size(3, 3), params["StereoRectifier.R_right"].as<std::vector<double>>());
    // set distortion parameters depending on the camera model
    const auto D_l_vec = params["StereoRectifier.D_left"].as<std::vector<double>>();
    const auto D_r_vec = params["StereoRectifier.D_right"].as<std::vector<double>>();
    const auto D_l = parse_vector_as_mat(cv::Size(1, D_l_vec.size()), D_l_vec);
    const auto D_r = parse_vector_as_mat(cv::Size(1, D_r_vec.size()), D_r_vec);
    // create camera matrix after rectification
    const cv::Matx33f K_rect = cv::Matx33f(params["Camera.fx"].as<float>(), 0, params["Camera.cx"].as<float>(), 
        0, params["Camera.fy"].as<float>(), params["Camera.cy"].as<float>(), 0, 0, 1);
    // create undistortion maps
    
    if(fisheye)
    {
        cv::fisheye::initUndistortRectifyMap(K_l, D_l, R_l, K_rect, img_size, CV_32F, undist_map_x_l_, undist_map_y_l_);
        cv::fisheye::initUndistortRectifyMap(K_r, D_r, R_r, K_rect, img_size, CV_32F, undist_map_x_r_, undist_map_y_r_);
    }
    else
    {
        cv::initUndistortRectifyMap(K_l, D_l, R_l, K_rect, img_size, CV_32F, undist_map_x_l_, undist_map_y_l_);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, K_rect, img_size, CV_32F, undist_map_x_r_, undist_map_y_r_);
    }
}

stereo_rectifier::~stereo_rectifier() {

}

void stereo_rectifier::rectify(const cv::Mat& in_img_l, const cv::Mat& in_img_r,
                               cv::Mat& out_img_l, cv::Mat& out_img_r) const {
    cv::remap(in_img_l, out_img_l, undist_map_x_l_, undist_map_y_l_, cv::INTER_LINEAR);
    cv::remap(in_img_r, out_img_r, undist_map_x_r_, undist_map_y_r_, cv::INTER_LINEAR);
}

void stereo_rectifier::rectify(const cv::Mat& in_img_l, cv::Mat& out_img_l) const {
    cv::remap(in_img_l, out_img_l, undist_map_x_l_, undist_map_y_l_, cv::INTER_LINEAR);
}

cv::Mat stereo_rectifier::parse_vector_as_mat(const cv::Size& shape, const std::vector<double>& vec) {
    cv::Mat mat(shape, CV_64F);
    std::memcpy(mat.data, vec.data(), shape.height * shape.width * sizeof(double));
    return mat;
}

} // namespace ORB_SLAM2
