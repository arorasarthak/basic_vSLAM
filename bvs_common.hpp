//
// Created by Sarthak Arora on 4/19/21.
//



#ifndef BASIC_VSLAM_BVS_COMMON_HPP
#define BASIC_VSLAM_BVS_COMMON_HPP

#include <opencv2/core.hpp>

namespace bvSLAM
{
    // Camera Intrinsics
    inline cv::Mat1d K = (cv::Mat1d(3,3) <<
            707.0493000000, 0.000000000000, 604.0814000000,
            0.000000000000, 707.0493000000, 180.5066000000,
            0.000000000000, 0.000000000000, 1.000000000000);

    inline cv::Mat1d K_3x4 = (cv::Mat1d(3,4) <<
            K.at<double>(0,0), K.at<double>(0,1), K.at<double>(0,2), 0.0,
            K.at<double>(1,0), K.at<double>(1,1), K.at<double>(1,2), 0.0,
            K.at<double>(2,0), K.at<double>(2,1), K.at<double>(2,2), 0.0);

}

#endif //BASIC_VSLAM_BVS_COMMON_HPP
