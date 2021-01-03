
#pragma once

#include "pointcloud.h"

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/StdVector>

void ReadXXX(const std::string& filename, std::vector<Eigen::Vector3d>& points,
                      std::vector<Eigen::Vector3d>& normals,
                      std::vector<Eigen::Vector3d>& colors);

void ReadSTL(const std::string& filename, std::vector<Eigen::Vector3d>& points,
             std::vector<Eigen::Vector3d>& normals,
             std::vector<Eigen::Vector3d>& colors);

void ReadPCD(const std::string& filename, std::vector<Eigen::Vector3d>& points,
             std::vector<Eigen::Vector3d>& normals,
             std::vector<Eigen::Vector3d>& colors);

void WritePCD(const PointCloud& pcd, const std::string& filename);

void ReadXYZM(const std::string& filename, std::vector<Eigen::Vector3d>& points,
              std::vector<Eigen::Vector3d>& normals,
              std::vector<Eigen::Vector3d>& colors);

void ReadPLY(const std::string& filename, std::vector<Eigen::Vector3d>& points,
              std::vector<Eigen::Vector3d>& normals,
              std::vector<Eigen::Vector3d>& colors);
