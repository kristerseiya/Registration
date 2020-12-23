
#include <stdio.h>
#include <stdlib.h>
#include <memory>
#include <iostream>
#include <string>

#include "open3d/geometry/PointCloud.h"
#include "open3d/visualization/utility/DrawGeometry.h"

using namespace open3d;

std::shared_ptr<geometry::PointCloud> read_pcd(char* filename) {

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

  if (fread(buffer, sizeof(float), 3, fp)==3) {
    pcd->colors_.push_back(Eigen::Vector3d(buffer[0],buffer[1],buffer[2]));
    for (unsigned long i = 1; i < size; i++) {
      fread(buffer, sizeof(float), 3, fp);
      pcd->colors_.push_back(Eigen::Vector3d(buffer[0],buffer[1],buffer[2]));
    }
  }

  fclose(fp);

  return pcd;
}

// bool isFloat(const std::string& input) {
//   std::string::const_iterator it = input.begin();
//   bool decimalPoint = false;
//   int minSize = 0;
//   if(input.size()>0 && (input[0] == '-' || input[0] == '+')){
//     it++;
//     minSize++;
//   }
//   while(it != input.end()){
//     if(*it == '.'){
//       if(!decimalPoint) decimalPoint = true;
//       else break;
//     }else if(!std::isdigit(*it) && ((*it!='f') || it+1 != input.end() || !decimalPoint)){
//       break;
//     }
//     ++it;
//   }
//   return input.size()>minSize && it == input.end();
// }

int main(int argc, char** argv) {

  std::vector< Eigen::Vector3d > colors(2);
  colors[0] = Eigen::Vector3d(1, 0.706, 0);
  colors[1] = Eigen::Vector3d(0, 0.651, 0.929);

  std::vector< std::shared_ptr<const geometry::Geometry> > geometries;
  // for (int i = 1; i < argc; i++) {
  //   std::shared_ptr<geometry::PointCloud> pcd = read_pcd(argv[i]);
  //   // pcd->PaintUniformColor(colors[i % 2]);
  //   geometries.push_back(pcd);
  // }

  char buffer[10];
  std::string input;
  int r, g, b;
  Eigen::Vector3d color;
  printf("Set color for the point clouds\n");
  printf("type \"custom\" to enter RGB value\n");
  printf("Hit ENTER without typing anything to use default color\n\n");
  for (int i = 1; i < argc; i++) {
    std::shared_ptr<geometry::PointCloud> pcd = read_pcd(argv[i]);
    printf("Color for Point Cloud #%d: ", i);
    fgets(buffer, 9, stdin);
    input.assign(buffer);
    if (input.size()==1) {
      /* use default */
    } else if (input.compare(0,3,"red")==0) {
      color[0] = 1.; color[1] = 0.; color[2] = 0.;
      pcd->PaintUniformColor(color);
    } else if (input.compare(0,4,"blue")==0) {
      color[0] = 0.; color[1] = 0.; color[2] = 1.;
      pcd->PaintUniformColor(color);
    } else if (input.compare(0,5,"green")==0) {
      color[0] = 0.; color[1] = 1.; color[2] = 0.;
      pcd->PaintUniformColor(color);
    } else if (input.compare(0,6,"custom")==0) {
      printf("Enter RGB numbers <space separated>: ");
      scanf("%d %d %d", &r, &g, &b);
      color[0] = r / 255.; color[1] = g / 255.; color[2] = b / 255.;
      pcd->PaintUniformColor(color);
    } else {
      printf("did not understand input\n");
      printf("using default color\n");
    }
    geometries.push_back(pcd);
    printf("\n");
  }

  visualization::DrawGeometries(geometries);
  exit(0);
}
