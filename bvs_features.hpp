//
// Created by Sarthak Arora on 4/19/21.
//

#ifndef BVS_FEATURES_HPP
#define BVS_FEATURES_HPP

#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include "bvs_common.hpp"


using Keypoints = std::vector<cv::KeyPoint>;
using Descriptors = cv::Mat;
using Image = cv::Mat;
using Corners = std::vector<cv::Point2f>;
using Pointset = std::vector<cv::Point2f>;
using Matches = std::vector<cv::DMatch>;

namespace bvSLAM {

    class bvs_features {

    public:

        bvs_features();

        Keypoints extract_keypoints(Image& gray_image);

        Descriptors compute_descriptors(Image& input_img,Keypoints &keypoints);

        Matches match_features(Descriptors& ds1, Descriptors& ds2);

        std::pair<Pointset, Pointset> match_points(Keypoints& kp1, Keypoints& kp2, Matches& matches);

        void plot_features(Image& input_img, Keypoints& input_keypoints);

        //void plot_matches();


        cv::Point2f p1, p2; // matched points p1 <==> p2
        Pointset ps1, ps2; // ps1 <==> ps2 -> represent correspondences

    private:

        Image bw_img {};
        Corners corner_features {};

        std::pair<Pointset, Pointset> correspondences {};

        bool plot_mode = true;

        const float ratio_thresh = 0.55f;
        std::vector<cv::DMatch> good_matches;
        std::vector< std::vector<cv::DMatch> > knn_matches;

        std::shared_ptr<cv::SiftDescriptorExtractor> d_extractor;
        std::shared_ptr<cv::DescriptorMatcher> d_matcher;
    };

}


#endif //BVS_FEATURES_HPP
