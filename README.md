library for exploring registration algorithms

must use c++11

depedencies:
eigen, flann, omp, jsoncpp

majority of the code taken from Open3D

Open3D is required for visualizer

tested on MacOS BigSur

run "make" to create dynamic library
run "make stitch" to build stitch app
run "./stitch config.json" to run registration
edit config.json file to change registration algorithms to use and their associated parameters
