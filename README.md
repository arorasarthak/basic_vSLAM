# basic_vSLAM
A basic frontend implementation of feature based visual SLAM

The code performs simple visual odometry using Shitomasi corners along with SIFT descriptors without any backend optimization. The code also displays Fundamental and Essential Matrices between every two consecutive frames. Two video files of the output have also been included. The C++ program generates a log file that is read by the python file for plotting purposes.

## Installation:

To use the code, it needs to be built using CMake. The CMake project requires OpenCV, Eigen and DBoW3 as dependencies.

mkdir build && cd build

cmake .. 

make

Usage:

./built_executable /full/path/to/the/first/image/in/the/sequence.png

References:

[1] https://docs.opencv.org/master/d9/df8/tutorial_root.html (C++ tutorials)
