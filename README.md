# Point Cloud Registration Library

This is a C++ library for exploring registration algorithms
C++11 is required

## Dependencies:
Eigen, flann, omp, jsoncpp

Open3D is required for visualizer

## Installation
run the following command to build the dynamic library
```bash
make
```

run the following command to build the stitch app (must build the dynamic library first)
```bash
make stitch
```

run the following command to run registration on pointclouds
```bash
./stitch config.json
```

edit config.json file to change registration algorithms to use and their associated parameters

## Acknowledgement
majority of the code taken from Open3D
tested on MacOS BigSur
