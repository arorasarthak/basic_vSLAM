//
// Created by Sarthak Arora on 4/17/21.
//

#include "bvs_frontend.hpp"
#include "bvs_odom.hpp"
#include <fstream>
#include <iostream>
#include "DBoW3/DBoW3.h"

bvSLAM::bvs_frontend::bvs_frontend(std::string path) {
    filepath = path;

    sequence = cv::VideoCapture(filepath, cv::CAP_IMAGES);

    if (!sequence.isOpened())
    {
        //  [TODO] throw exception here
        std::cerr << "Failed to open Image Sequence!\n" << std::endl;
    }
    rng = cv::RNG(12345);

}

void bvSLAM::bvs_frontend::run() {

    std::ofstream myfile;
    myfile.open ("log.txt");

    DBoW3::Vocabulary vocab;
    std::vector<cv::Mat> vocab_ds;
    int vctr {0};

    auto f = bvSLAM::bvs_features();
    auto camera_odom = bvSLAM::bvs_odom();

    Image img1, img2;
    Image traj_img = cv::Mat::zeros(500, 500, CV_8UC3);
    Image pt_img = cv::Mat::zeros(500, 500, CV_8UC3);
//
    cv::Mat R1, R2, t1;
//
    cv::Mat H = cv::Mat_<double>(4,4);
    cv::Mat camera_mat1 = cv::Mat_<double>(3,4);
    cv::Mat camera_mat2 = cv::Mat_<double>(3,4);
    auto Rt = camera_mat2.clone();
    cv::Mat triangulated_points;
    cv::Mat world_points;
    cv::Mat world_pt = cv::Mat_<double>(4,1);
    std::vector<cv::Mat> point_vec;

    H.at<double>(0,0) = 1;
    H.at<double>(1,1) = 1;
    H.at<double>(2,2) = 1;
    H.at<double>(3,3) = 1;

    camera_mat1.at<double>(0,0) = 1;
    camera_mat1.at<double>(1,1) = 1;
    camera_mat1.at<double>(2,2) = 1;

    camera_mat1 = K * camera_mat1;

    cv::Mat camera_pose = H.clone();


    cv::Mat fundamental_matrix;
    cv::Mat essential_matrix;

    img1 = this->get_next_image();  // Store the previous image

    while(!seq_is_empty) {

        img2 = this->get_next_image(); // Store the current image

        kp1 = f.extract_keypoints(img1);
        kp2 = f.extract_keypoints(img2);

        ds1 = f.compute_descriptors(img1, kp1);
        ds2 = f.compute_descriptors(img2, kp2);

        matches = f.match_features(ds1, ds2);

        p1p2 = f.match_points(kp1, kp2, matches);

        plot_features(img1, kp1, "win1");
        plot_matches(p1p2.first, p1p2.second, img2, "Point Matches");

        fundamental_matrix = cv::findFundamentalMat(p1p2.first, p1p2.second);

        essential_matrix = cv::findEssentialMat(p1p2.first, p1p2.second, K, cv::RANSAC);

        std::cerr << "Fundamental Matrix: " << fundamental_matrix << '\n';
        std::cerr << "Essential Matrix: " << essential_matrix << '\n';

        cv::recoverPose(essential_matrix,p1p2.first, p1p2.second, K, R1, t1);


        H.at<double>(0,0) = R1.at<double>(0,0);
        H.at<double>(0,1) = R1.at<double>(0,1);
        H.at<double>(0,2) = R1.at<double>(0,2);

        H.at<double>(1,0) = R1.at<double>(1,0);
        H.at<double>(1,1) = R1.at<double>(1,1);
        H.at<double>(1,2) = R1.at<double>(1,2);

        H.at<double>(2,0) = R1.at<double>(2,0);
        H.at<double>(2,1) = R1.at<double>(2,1);
        H.at<double>(2,2) = R1.at<double>(2,2);

        H.at<double>(0,3) = t1.at<double>(0);
        H.at<double>(1,3) = t1.at<double>(1);
        H.at<double>(2,3) = t1.at<double>(2);

        for(size_t i {0}; i < H.rows - 1 ;++i){
            H.row(i).copyTo(Rt.row(i));
        }


        camera_pose = camera_pose * H; // relative transformation

        camera_mat2 = K * Rt;

        cv::triangulatePoints(camera_mat1, camera_mat2, p1p2.first, p1p2.second, triangulated_points);

        world_points = cv::Mat::ones( triangulated_points.size(), CV_32FC1 );

        for (size_t i = 0; i < world_points.cols; ++i){
            world_points.col(i).at<double>(0) = triangulated_points.col(i).at<double>(0);
            world_points.col(i).at<double>(1) = triangulated_points.col(i).at<double>(1);
            world_points.col(i).at<double>(2) = triangulated_points.col(i).at<double>(2);
            world_points.col(i).copyTo(world_pt.col(0));
            point_vec.emplace_back(camera_pose * world_pt);
            auto odom_pt = cv::Point3f(camera_pose.at<double>(0,3), camera_pose.at<double>(1,3), camera_pose.at<double>(2,3));
            myfile << (camera_pose * world_pt).t() << ',' << odom_pt << ";\n";
        }


        plot_traj(cv::Point2f(camera_pose.at<double>(0,3), camera_pose.at<double>(2,3)), traj_img);
//        camera_odom = compute_camera_pose(img1, img2);

        if (vctr % 10 == 0) {
            vocab_ds.emplace_back(ds1);
        }

        if (vctr == 700) {
            std::cerr << "Saving vocab \n";
//            vocab.create(vocab_ds);
//            vocab.save("vocab");
        }

        vctr = vctr + 1;
        img1 = img2;


    }
    std::cerr << "Closing!!" << '\n';
    myfile.close();
}

cv::Mat bvSLAM::bvs_frontend::get_next_image() {
    Image img;
    sequence >> img;
    if (!img.empty()){
        return img;
    } else {
        seq_is_empty = true;
        std::cerr << "End of sequence!!" << '\n';
    }
}

void bvSLAM::bvs_frontend::plot_features(Image &img, Keypoints &kp, std::string window_name) {

        Image out_img;

        cv::drawKeypoints(img, kp, out_img,cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::namedWindow(window_name);
        cv::imshow(window_name, out_img);
        //window_index = window_index + 1;

        if (cv::waitKey(10) == 113) {
            //break;
            cv::destroyWindow(window_name);
        }
}

void bvSLAM::bvs_frontend::plot_matches(Pointset &ps1, Pointset &ps2, Image &image, std::string window_name) {

    Image out_img = image.clone();
    cv::Scalar color {};
    for (size_t i {0}; i <= ps1.size(); ++i) {
//        color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
        color = cv::Scalar(0,0,255);
        cv::line(out_img, ps2[i], ps1[i], color,1, cv::LINE_AA);
        cv::circle(out_img, ps2[i], 3, color, 1, 8, 0);
    }

    cv::namedWindow(window_name);
    cv::imshow(window_name, out_img);

    if (cv::waitKey(10) == 113) {
        //break;
        cv::destroyWindow(window_name);
    }
}

void bvSLAM::bvs_frontend::plot_traj(cv::Point2f traj_pt, Image& traj_img) {

    std::string window_name = "Trajectory Plot";
    traj_pt.x = traj_pt.x + 200;
    traj_pt.y = traj_pt.y + 200;
    cv::circle(traj_img, traj_pt,2,cv::Scalar(0,255,0));


    cv::namedWindow(window_name);
    cv::imshow(window_name, traj_img);

    if (cv::waitKey(10) == 113) {
        //break;
        cv::destroyWindow(window_name);
    }

}

void bvSLAM::bvs_frontend::plot_points(std::vector<cv::Mat>& pointcloud, Image &input_img){

    auto pt = cv::Point2d();

    for (size_t i = 0; i < pointcloud.size(); ++i){
        pt.x = pointcloud.at(i).at<double>(0) + 800;
        pt.y = pointcloud.at(i).at<double>(2) + 800;
        cv::circle(input_img, pt,0.05,cv::Scalar(255,255,255));
    }

}

bvSLAM::bvs_odom bvSLAM::bvs_frontend::compute_camera_pose(Image &img1, Image &img2) {
    std::cerr << "boom\n";
    kp1 = feature_extractor.extract_keypoints(img1);
    std::cerr << "boom\n";
    kp2 = feature_extractor.extract_keypoints(img2);

    ds1 = feature_extractor.compute_descriptors(img1, kp1);
    ds2 = feature_extractor.compute_descriptors(img2, kp2);

    matches = feature_extractor.match_features(ds1, ds2);

    p1p2 = feature_extractor.match_points(kp1, kp2, matches);
    std::cerr << "boom\n";
    E = cv::findEssentialMat(p1p2.first, p1p2.second, K, cv::RANSAC);

    cv::recoverPose(E, p1p2.first, p1p2.second, K, odom.R, odom.t);
    std::cerr << "boom\n";
    odom.update();
    std::cerr << "boom\n";
    // TODO implement odometry update

    // compute descriptors
    // compute matches
    // compute essential matrix
    // compute pose from essential matrix

    return odom;
}

std::vector<cv::Point3d> bvSLAM::bvs_frontend::compute_3d_points() {
    return std::vector<cv::Point3d>();
}

