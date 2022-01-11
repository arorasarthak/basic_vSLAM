cmake_minimum_required(VERSION 3.17)
project(basic_vSLAM)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(DBoW3)

add_executable(basic_vSLAM bvslam.cpp bvs_frontend.cpp bvs_frontend.hpp bvs_features.cpp bvs_features.hpp bvslam.cpp bvs_common.hpp bvs_odom.cpp bvs_odom.hpp)
target_link_libraries(basic_vSLAM Eigen3::Eigen ${OpenCV_LIBS}  ${DBoW3_LIBS})
