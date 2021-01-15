# Point Cloud Registration Library

This is a C++ library for exploring registration algorithms. \
Supports ICP (point-to-point and point-to-plane), Colored ICP, RANSAC (correspondence based and feature based), Fast Global Registration. \
C++11 is required

## Dependencies:
Eigen, flann, omp

## Installation
First, edit the CMakeLists.txt \
Run the following command to build the dynamic library
```bash
mkdir build && cd build && cmake ..
```

## Usage
Include ```pointcloud.h``` and ```io.h``` to read point clouds from .ply and .stl files. Include ```registration.h``` to perform registration. Include ```feature.h``` to compute FPFH features of a point cloud.

run the following command to run registration on pointclouds
```bash
./stitch config.json
```

edit config.json file to change registration algorithms to use and their associated parameters

## Acknowledgement
majority of the code taken from Open3D
tested on MacOS BigSur
