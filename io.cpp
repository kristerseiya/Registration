
#include "io.h"

#include "pointcloud.h"

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/StdVector>

std::vector<Eigen::Vector3d> read_stl(std::string path) {

  FILE* fp = fopen(path.c_str(),"rb");
  if (fp == NULL) {
    printf("failed to open file\n");
    exit(1);
  }
  char header[80];
  fread(header,sizeof(char),80,fp);
  unsigned int n_face;
  fread(&n_face, sizeof(unsigned int), 1, fp);
  size_t n_points = ((size_t)n_face) * 3;

  std::vector<Eigen::Vector3d> points(n_points);
  // normals.clear();
  // points = new float[((size_t)n_face) * 3];
  // normals = new float[((size_t)n_face) * 9];
  // short attr;
  float buffer[9];

  for (size_t i = 0; i < n_face; i++) {
    fread(buffer, sizeof(float), 3, fp);
    fread(buffer, sizeof(float), 9, fp);
    points[i*3+0] = Eigen::Vector3d(buffer[0], buffer[1], buffer[2]);
    points[i*3+1] = Eigen::Vector3d(buffer[3], buffer[4], buffer[5]);
    points[i*3+2] = Eigen::Vector3d(buffer[6], buffer[7], buffer[8]);
    fread(buffer, sizeof(short), 1, fp);
  }

  fclose(fp);

//   printf("successfully read stl file\n");
  return points;
}

std::vector<Eigen::Vector3d> read_pts(std::string filename) {

  FILE* fp = fopen(filename.c_str(), "rb");
  if (fp == NULL) {
    exit(1);
  }

  unsigned long size;
  fread(&size, sizeof(unsigned long), 1, fp);
  std::vector<Eigen::Vector3d> points(size);
  float buffer[3];
  for (unsigned long i = 0; i < size; i++) {
    fread(buffer, sizeof(float), 3, fp);
    points[i] = Eigen::Vector3d(buffer[0],buffer[1],buffer[2]);
  }
  fclose(fp);

  // std::shared_ptr<geometry::PointCloud> pcd = std::make_shared<geometry::PointCloud>();
  // pcd->points_ = points;
  return points;
}

void write_pts(const PointCloud& pcd, std::string filename) {
  FILE* fp = fopen(filename.c_str(), "wb");
  unsigned long size = pcd.points_.size();
  fwrite(&size, sizeof(unsigned long), 1, fp);
  for (auto point : pcd.points_) {
	float buffer[3];
	buffer[0] = point[0]; buffer[1] = point[1]; buffer[2] = point[2];
    fwrite(buffer, sizeof(float), 3, fp);
  }
  fclose(fp);
  return;
}

//
// void read_ply(char* path, size_t* n_points, float* points, float* normals, unsigned char* colors) {
//
//   char line[101];
//   float pt[3];
//   float n_pt[3];
//   unsigned char color[3];
//
//   FILE* ply_file = fopen(path,"rb");
//
//   if (ply_file == NULL) {
//     printf("failed to open file\n");
//     exit(1);
//   }
//
//   for (int i = 0; i < 13; i++) {
//     fgets(line,100,ply_file);
//   }
//
//   *n_points = 157006;
//   points = new float[3* (*n_points)];
//   normals = new float[3*(*n_points)];
//   colors = new unsigned char[3*(*n_points)];
//
//   size_t i = 0;
//   while (!feof(ply_file)) {
//     fread(pt,sizeof(float),3,ply_file);
//     fread(n_pt,sizeof(float),3,ply_file);
//     fread(color,sizeof(unsigned char),3,ply_file);
//
//     if (!feof(ply_file)) {
//       unsigned char bytes[4];
//       for (int i = 0; i < 3; i++) {
//         memcpy(bytes,&pt[i],4);
//         int big_endian = bytes[0] | (bytes[1]<<8) | (bytes[2]<<16) | (bytes[3]<<24);
//         memcpy(&pt[i],&big_endian,4);
//         memcpy(bytes,&(n_pt[i]),4);
//         big_endian = bytes[0] | (bytes[1]<<8) | (bytes[2]<<16) | (bytes[3]<<24);
//         memcpy(&n_pt[i],&big_endian,4);
//       }
//       memcpy(points+3*i,pt,3*sizeof(float));
//       memcpy(normals+3*i,n_pt,3*sizeof(float));
//       memcpy(colors+3*i,color,3*sizeof(unsigned char));
//       i++;
//     }
//   }
//
//   fclose(ply_file);
//
// }
//
// void read_xyzm(char* path, size_t* n_points, float* points, float* normals, unsigned char* colors) {
//
//   FILE* xyzm_file = fopen(path,"rb");
//   if (xyzm_file == NULL) {
//     printf("failed to open file\n");
//     exit(1);
//   }
//   int height;
//   int width;
//   int ret = fscanf(xyzm_file, "image size width x height = %d x %d",&width,&height);
//   if (ret!=2) {
//     printf("unsupported file format\n");
//     fclose(xyzm_file);
//     exit(1);
//   }
//   *n_points = width * height;
//   while(fgetc(xyzm_file)==0);
//   fseek(xyzm_file,-1,SEEK_CUR);
//   points = new float[3*width*height];
//   colors = new unsigned char[3*width*height];
//   // unsigned char* mask = new unsigned char[width*height];
//   fread(points,sizeof(float),3*width*height,xyzm_file);
//   fread(colors,sizeof(unsigned char),3*width*height,xyzm_file);
//   // fread(mask,sizeof(unsigned char),width*height,xyzm_file);
//   fclose(xyzm_file);
//   // this->mask = mask;
//   printf("successfully read xyzm file\n");
// }
