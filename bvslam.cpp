//
// Created by Sarthak Arora on 4/19/21.
//

#include "bvs_frontend.hpp"

int main(int argc, char** argv){

    auto frontend = bvSLAM::bvs_frontend(argv[1]);
    std::cout << "Running Sequence: " << argv[1] << '\n';
    frontend.run();

    return 0;
}