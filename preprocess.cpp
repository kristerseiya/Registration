#include "io.h"
#include "pointcloud.h"

#include <stdlib.h>
#include <iostream>
#include <tuple>

std::vector<Eigen::Vector3d> read_xyzm(std::string filename, float threshold) {

  FILE* fp = fopen(filename.c_str(),"rb");
  if (fp == NULL) {
    fprintf(stderr,"failed to open file\n");
    exit(1);
  }
  int height;
  int width;
  int ret = fscanf(fp, "image size width x height = %d x %d", &width, &height);
  if (ret!=2) {
    printf("unsupported file format\n");
    fclose(fp);
    exit(1);
  }
  while(fgetc(fp)==0);
  fseek(fp,-1,SEEK_CUR);
  std::vector<Eigen::Vector3d> points(width*height);
  float buffer[3];
  for (size_t i = 0; i < (size_t)width*height; i++) {
    fread(buffer, sizeof(float), 3, fp);
    if  ((std::abs(buffer[0]) > threshold) ||
         (std::abs(buffer[1]) > threshold) ||
         (std::abs(buffer[2]) > threshold)) {
           continue;
    }
    points[i] = Eigen::Vector3d(buffer[0], buffer[1], buffer[2]);
  }
  fclose(fp);

  return points;
}

int main(int argc, char** argv) {
  PointCloud pcd(read_xyzm(argv[1], 700));
  pcd = pcd.RemoveNonFinitePoints();
  auto x = pcd.RemoveStatisticalOutliers(200,2);
  auto processed_pcd = std::get<0>(x);
  write_pts(*processed_pcd, "xyzm.pts");
  exit(0);
}
