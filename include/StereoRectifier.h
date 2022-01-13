#ifndef ORB_SLAM2_STEREO_RECTIFIER_H
#define ORB_SLAM2_STEREO_RECTIFIER_H

#include <opencv2/core.hpp>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>


namespace ORB_SLAM2 {

class stereo_rectifier {
public:

    //! Constructor
    stereo_rectifier(const std::string &strSettingPath, const bool fisheye);

    //! Destructor
    virtual ~stereo_rectifier();

    //! Apply stereo-rectification
    void rectify(const cv::Mat& in_img_l, const cv::Mat& in_img_r,
                 cv::Mat& out_img_l, cv::Mat& out_img_r) const;

    void rectify(const cv::Mat& in_img_l, cv::Mat& out_img_l) const;

private:
    //! Parse std::vector as cv::Mat
    static cv::Mat parse_vector_as_mat(const cv::Size& shape, const std::vector<double>& vec);

    //! undistortion map for x-axis in left image
    cv::Mat undist_map_x_l_;
    //! undistortion map for y-axis in left image
    cv::Mat undist_map_y_l_;
    //! undistortion map for x-axis in right image
    cv::Mat undist_map_x_r_;
    //! undistortion map for y-axis in right image
    cv::Mat undist_map_y_r_;
};

} // namespace ORB_SLAM2

#endif // ORB_SLAM2_STEREO_RECTIFIER_H
