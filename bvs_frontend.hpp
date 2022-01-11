//
// Created by Sarthak Arora on 4/17/21.
//

#ifndef BVS_FRONTEND_HPP
#define BVS_FRONTEND_HPP


#include "bvs_features.hpp"
#include "bvs_odom.hpp"
#include <opencv2/calib3d.hpp>

namespace bvSLAM {
    class bvs_frontend {
    public:
        bvs_frontend(std::string path);

        cv::Mat get_next_image();

        bvs_odom compute_camera_pose(Image& img1, Image& img2);

        std::vector<cv::Point3d> compute_3d_points();

        void plot_features(Image& img, Keypoints& kp, std::string window_name);

        void plot_matches(Pointset& ps1, Pointset& ps2, Image& image, std::string window_name);

        void plot_traj(cv::Point2f traj_pt, Image& traj_img);

        void plot_points(std::vector<cv::Mat>& pointcloud, Image& input_img);

        void run();

    private:
        std::string filepath; // path to img sequence
        cv::VideoCapture sequence;
        cv::Mat img;
        bool seq_is_empty {false};
        cv::RNG rng;

        bvs_odom odom;

        bvs_features feature_extractor;
        Keypoints kp1;
        Keypoints kp2;
        Descriptors ds1;
        Descriptors ds2;
        Matches matches;

        std::pair<Pointset, Pointset> p1p2;

        cv::Mat E; // essential matrix

    };
}

#endif //BVS_FRONTEND_HPP
