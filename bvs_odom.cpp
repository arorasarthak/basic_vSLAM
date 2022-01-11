//
// Created by Sarthak Arora on 4/19/21.
//

#include "bvs_odom.hpp"
#include <cmath>

void bvSLAM::bvs_odom::update() {

    transform.at<double>(0,0) = R.at<double>(0,0);
    transform.at<double>(0,1) = R.at<double>(0,1);
    transform.at<double>(0,2) = R.at<double>(0,2);

    transform.at<double>(1,0) = R.at<double>(1,0);
    transform.at<double>(1,1) = R.at<double>(1,1);
    transform.at<double>(1,2) = R.at<double>(1,2);

    transform.at<double>(2,0) = R.at<double>(2,0);
    transform.at<double>(2,1) = R.at<double>(2,1);
    transform.at<double>(2,2) = R.at<double>(2,2);

    transform.at<double>(0,3) = t.at<double>(0);
    transform.at<double>(1,3) = t.at<double>(1);
    transform.at<double>(2,3) = t.at<double>(2);

//    pose = pose * transform; // relative transformation

//    pose_vec.at(0) = pose.at<double>(0,3);
//    pose_vec.at(0) = pose.at<double>(1,3);
//    pose_vec.at(0) = pose.at<double>(2,3);

//    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
//
//    bool singular = sy < 1e-6; // If
//
//    double roll, pitch, yaw;
//    if (!singular)
//    {
//        roll = atan2(R.at<double>(2,1) , R.at<double>(2,2));
//        pitch = atan2(-R.at<double>(2,0), sy);
//        yaw = atan2(R.at<double>(1,0), R.at<double>(0,0));
//    }
//    else
//    {
//        roll = atan2(-R.at<double>(1,2), R.at<double>(1,1));
//        pitch = atan2(-R.at<double>(2,0), sy);
//        yaw = 0;
//    }
//
//    pose_vec.at(3) = roll;
//    pose_vec.at(4) = pitch;
//    pose_vec.at(5) = yaw;

}
