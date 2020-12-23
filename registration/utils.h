
#pragma once

#include <Eigen/Core>

#include "pointcloud.h"

Eigen::Matrix4d generate_random_rotation(double rx, double ry, double rz, double max_offset);

PointCloud& add_noise(PointCloud& pcd, double sigma);
