
#include <stdio.h>
#include <stdlib.h>
#include <memory>
#include <iostream>

#include "open3d/geometry/PointCloud.h"
#include "open3d/visualization/utility/DrawGeometry.h"

using namespace open3d;

std::shared_ptr<geometry::PointCloud> read_pts(char* filename) {

  FILE* fp = fopen(filename, "rb");
  if (fp == NULL) {
    exit(1);
  }

  unsigned long size;
  fread(&size, sizeof(unsigned long), 1, fp);

  std::shared_ptr<geometry::PointCloud> pcd = std::make_shared<geometry::PointCloud>();
  float buffer[3];
  for (unsigned long i = 0; i < size; i++) {
    fread(buffer, sizeof(float), 3, fp);
    pcd->points_.push_back(Eigen::Vector3d(buffer[0],buffer[1],buffer[2]));
  }
  fclose(fp);

  // std::shared_ptr<geometry::PointCloud> pcd = std::make_shared<geometry::PointCloud>();
  // pcd->points_ = points;

  return pcd;
}

int main(int argc, char** argv) {

  std::vector< Eigen::Vector3d > colors(2);
  colors[0] = Eigen::Vector3d(1, 0.706, 0);
  colors[1] = Eigen::Vector3d(0, 0.651, 0.929);

  std::vector< std::shared_ptr<const geometry::Geometry> > geometries;
  for (int i = 1; i < argc; i++) {
    std::shared_ptr<geometry::PointCloud> pcd = read_pts(argv[i]);
    pcd->PaintUniformColor(colors[i % 2]);
    geometries.push_back(pcd);
  }

// 	auto pcd = read_pts(argv[1]);
// 	for (int i = 0; i < 5; i++) {
// 		std::cout << pcd->points_[i].transpose() << std::endl;
// 	}

// 	geometries.push_back(pcd);
  visualization::DrawGeometries(geometries);
  exit(0);
}
