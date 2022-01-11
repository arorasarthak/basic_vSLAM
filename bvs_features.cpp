//
// Created by Sarthak Arora on 4/19/21.
//

#include "bvs_features.hpp"

Keypoints bvSLAM::bvs_features::extract_keypoints(Image &input_img) {

    Keypoints bvs_keypoints;
    Corners corner_features;
    Image bw_img;

    cv::cvtColor(input_img, bw_img, cv::COLOR_RGB2GRAY);
    cv::goodFeaturesToTrack(bw_img, corner_features, 1500, 0.01, 10, cv::noArray(), 3,3);

    for(auto corner: corner_features)
    {
        auto kp = cv::KeyPoint(corner,2);
        bvs_keypoints.emplace_back(cv::KeyPoint(kp));
    }

    return bvs_keypoints;
}

Descriptors bvSLAM::bvs_features::compute_descriptors(Image& input_img,Keypoints &keypoints) {
    Descriptors bvs_descriptors;

    d_extractor->compute(input_img, keypoints, bvs_descriptors);

    return bvs_descriptors;
}

bvSLAM::bvs_features::bvs_features() {
    d_extractor = cv::SiftDescriptorExtractor::create();
    d_matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
}

void bvSLAM::bvs_features::plot_features(Image& input_img, Keypoints& input_keypoints) {

    if (plot_mode == true) {

        Image out_img;

        cv::drawKeypoints(input_img, input_keypoints, out_img);

        //cv::imshow("Feature View", out_img);
        cv::namedWindow("Feature Plotting Window");

        cv::imshow("Feature Plotting Window", out_img);

        if (cv::waitKey(50) == 113) {
            //break;
            plot_mode = false;
            cv::destroyWindow("Feature Plotting Window");
        }
    }
}

Matches bvSLAM::bvs_features::match_features(Descriptors &ds1, Descriptors &ds2) {

    Matches good_matches {};
    std::vector<Matches> knn_matches;

    d_matcher->knnMatch(ds1 , ds2, knn_matches,2);
    // Filter on the basis of quality??
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }
    return good_matches;
}

std::pair<Pointset, Pointset> bvSLAM::bvs_features::match_points(Keypoints &kp1, Keypoints &kp2, Matches& matches) {

    cv::Point2f p1, p2;
    Pointset pointset1 {};
    Pointset pointset2 {};

    for(auto match: matches) {
        p2 = kp1[match.queryIdx].pt; // current
        pointset2.emplace_back(p2);

        p1 = kp2[match.trainIdx].pt; // old
        pointset1.emplace_back(p1);
    }
    correspondences.first = pointset1;
    correspondences.second = pointset2;

    return correspondences;
}
