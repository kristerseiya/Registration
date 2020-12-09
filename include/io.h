
#pragma once

#include "pointcloud.h"

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/StdVector>

std::vector<Eigen::Vector3d> read_stl(std::string path);
std::vector<Eigen::Vector3d> read_pts(std::string filename);
void write_pts(const PointCloud& pcd, std::string filename);
// void read_ply(char* path, size_t* n_points, float* points, float* normals, unsigned char* colors);
// void read_xyzm(char* path, size_t* n_points, float* points, float* normals, unsigned char* colors);
