
#pragma once

#include "pointcloud.h"

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/StdVector>

std::vector<Eigen::Vector3d> ReadPointsFromFile(std::string filename);
std::vector<Eigen::Vector3d> read_stl(std::string filename);
std::vector<Eigen::Vector3d> read_pcd(std::string filename);
void write_pcd(const PointCloud& pcd, std::string filename);
std::vector<Eigen::Vector3d> read_xyzm(std::string filename);
// void read_ply(char* path, size_t* n_points, float* points, float* normals, unsigned char* colors);
// void read_xyzm(char* path, size_t* n_points, float* points, float* normals, unsigned char* colors);
