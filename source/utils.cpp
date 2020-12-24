
#include <Eigen/Core>
#include <random>
#include <math.h>
#include <stdlib.h>
#include <string>

#include "pointcloud.h"

#include "utils.h"

Eigen::Matrix4d GenerateRandomRotation(double rx, double ry, double rz, double max_offset) {
  unsigned seed = std::chrono::steady_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_real_distribution<double> distribution(0.0,1.0);
  double x_degree = (distribution(generator) - 0.5) * (2 * M_PI) / 360.0 * rx * 2;
  double y_degree = (distribution(generator) - 0.5) * (2 * M_PI) / 360.0 * ry * 2;
  double z_degree = (distribution(generator) - 0.5) * (2 * M_PI) / 360.0 * rz * 2;

  Eigen::Matrix3d r_x = Eigen::Matrix3d::Zero();
  r_x(0,0) = 1.0;
  r_x(1,1) = cos(x_degree);
  r_x(2,1) = sin(x_degree);
  r_x(1,2) = -sin(x_degree);
  r_x(2,2) = cos(x_degree);

  Eigen::Matrix3d r_y = Eigen::Matrix3d::Zero();
  r_y(1,1) = 1.0;
  r_y(0,0) = cos(y_degree);
  r_y(2,0) = -sin(y_degree);
  r_y(0,2) = sin(y_degree);
  r_y(2,2) = cos(y_degree);

  Eigen::Matrix3d r_z = Eigen::Matrix3d::Zero();
  r_z(2,2) = 1.0;
  r_z(0,0) = cos(z_degree);
  r_z(1,0) = sin(z_degree);
  r_z(0,1) = -sin(z_degree);
  r_z(1,1) = cos(z_degree);

  Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
  Eigen::Matrix3d R3 = r_x * r_y * r_z;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R(i,j) = R3(i,j);
    }
  }

  if (max_offset != 0.0) {
    for (int i = 0; i < 3; i++) {
      R(i,3) = distribution(generator) * max_offset * 2 - max_offset;
    }
  }

  R(3,3) = 1.0;

  return R;
}

PointCloud& AddNoise(PointCloud& pcd, double sigma) {

  unsigned seed = std::chrono::steady_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::normal_distribution<double> distribution(0.0,sigma);

  for (size_t i = 0; i < pcd.points_.size(); i++) {
      for (int j = 0; j < 3; j++) {
          pcd.points_[i][j] += distribution(generator);
      }
  }

  return pcd;
}
