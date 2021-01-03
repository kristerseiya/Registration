
#pragma once

#include <Eigen/Core>

#include "pointcloud.h"

Eigen::Matrix4d GenerateRandomRotation(double rx, double ry, double rz, double max_offset);

PointCloud& AddNoise(PointCloud& pcd, double sigma);
