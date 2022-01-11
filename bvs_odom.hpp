//
// Created by Sarthak Arora on 4/19/21.
//

#ifndef BVS_ODOM_HPP
#define BVS_ODOM_HPP

#include <opencv2/core.hpp>
#include <array>

namespace bvSLAM {

    class bvs_odom {
        public:

            bvs_odom() = default;

            void update();

            cv::Mat t; // (3x1) Vector translation vector to represent the camera 3-D position
            cv::Mat R; // (3x3) Matrix rotation matrix to represent the camera rotation
            cv::Mat transform;  // (4x4) Matrix tf between camera frames
            cv::Mat pose;  // (4x4 Matrix) complete pose of the camera in homogeneous form
            cv::Mat camera_matrix; // (3x4 Matrix) intrinsic + extrinsic matrix
            std::array<double, 6> pose_vec;
        };
}

#endif //BVS_ODOM_HPP
