
#include "io.h"

#include "pointcloud.h"

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/StdVector>

void ReadXXX(const std::string& filename, std::vector<Eigen::Vector3d>& points,
                      std::vector<Eigen::Vector3d>& normals,
                      std::vector<Eigen::Vector3d>& colors) {
  if (filename.compare(filename.size()-4,4,".stl")==0) {
    ReadSTL(filename, points, normals, colors);
  } else if (filename.compare(filename.size()-5,5,".xyzm")==0) {
    ReadXYZM(filename, points, normals, colors);
  } else if (filename.compare(filename.size()-4,4,".pts")==0) {
    ReadPCD(filename, points, normals, colors);
  } else if (filename.compare(filename.size()-4,4,".pcd")==0) {
    ReadPCD(filename, points, normals, colors);
  } else if (filename.compare(filename.size()-4,4,".ply")==0) {
	ReadPLY(filename, points, normals, colors);
  } else {
    // fprintf(stderr,"could not recognize file extension\n");
    // exit(1);
    throw std::runtime_error("could not recognize file extension");
  }
  return;
}

void ReadSTL(const std::string& filename, std::vector<Eigen::Vector3d>& points,
             std::vector<Eigen::Vector3d>& normals,
             std::vector<Eigen::Vector3d>& colors) {

  FILE* fp = fopen(filename.c_str(),"rb");
  if (fp == NULL) {
    // printf("failed to open file\n");
    // exit(1);
    throw std::runtime_error("failed to open file");
  }
  char header[80];
  fread(header,sizeof(char),80,fp);
  unsigned int n_face;
  fread(&n_face, sizeof(unsigned int), 1, fp);
  size_t n_points = ((size_t)n_face) * 3;

  points.clear();
  normals.clear();
  colors.clear();
  points.resize(n_points);

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

  return;
}

void ReadPCD(const std::string& filename, std::vector<Eigen::Vector3d>& points,
             std::vector<Eigen::Vector3d>& normals,
             std::vector<Eigen::Vector3d>& colors) {

  FILE* fp = fopen(filename.c_str(), "rb");
  if (fp == NULL) {
    // fprintf(stderr, "Could not read file %s\n", filename.c_str());
    // exit(1);
    throw std::runtime_error("could not read .pcd file");
  }

  unsigned long size;
  fread(&size, sizeof(unsigned long), 1, fp);

  points.clear();
  normals.clear();
  colors.clear();
  points.resize(size);

  float buffer[3];
  for (unsigned long i = 0; i < size; i++) {
    fread(buffer, sizeof(float), 3, fp);
    points[i] = Eigen::Vector3d(buffer[0],buffer[1],buffer[2]);
  }

  if (fread(buffer, sizeof(float), 3, fp)==3) {
    colors.resize(size);
    colors[0] = Eigen::Vector3d(buffer[0],buffer[1],buffer[2]);
    for (unsigned long i = 1; i < size; i++) {
      fread(buffer, sizeof(float), 3, fp);
      colors[i] = Eigen::Vector3d(buffer[0],buffer[1],buffer[2]);
    }
  }

  fclose(fp);

  return;
}

void WritePCD(const PointCloud& pcd, const std::string& filename) {
  FILE* fp = fopen(filename.c_str(), "wb");
  unsigned long size = pcd.points_.size();
  fwrite(&size, sizeof(unsigned long), 1, fp);
  float buffer[3];
  for (auto point : pcd.points_) {
	  buffer[0] = point[0]; buffer[1] = point[1]; buffer[2] = point[2];
    fwrite(buffer, sizeof(float), 3, fp);
  }
  for (auto color : pcd.colors_) {
    buffer[0] = color[0]; buffer[1] = color[1]; color[2] = buffer[2];
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

void ReadXYZM(const std::string& filename, std::vector<Eigen::Vector3d>& points,
              std::vector<Eigen::Vector3d>& normals,
              std::vector<Eigen::Vector3d>& colors) {

  FILE* fp = fopen(filename.c_str(),"rb");
  if (fp == NULL) {
    // fprintf(stderr,"failed to open file\n");
    // exit(1);
    throw std::runtime_error("failed to open .xyzm file");
  }
  int height;
  int width;
  int ret = fscanf(fp, "image size width x height = %d x %d", &width, &height);
  if (ret!=2) {
    // printf("unsupported file format\n");
    // fclose(fp);
    // exit(1);
    fclose(fp);
    throw std::runtime_error("unsupported file format");
  }
  while(fgetc(fp)==0);
  fseek(fp,-1,SEEK_CUR);
  points.clear();
  normals.clear();
  colors.clear();
  points.resize(width*height);
  float buffer[3];
  for (size_t i = 0; i < (size_t)width*height; i++) {
    fread(buffer, sizeof(float), 3, fp);
    points[i] = Eigen::Vector3d(buffer[0], buffer[1], buffer[2]);
  }
  fclose(fp);

  return;
}

static void swap_bytes(char* x, int size) {
    char* tmp = (char*)malloc(sizeof(char)*size);
    for (int i = 0; i < size; i++) {
        tmp[i] = x[size-1-i];
    }
    for (int i = 0; i < size; i++) {
        x[i] = tmp[i];
    }
    free(tmp);
}

static bool is_little_endian() {
    int x = 1;
    char* y = (char*)malloc(sizeof(int));
    memcpy(y,&x,sizeof(int));
    char z = y[0];
    free(y);
    return z;
}

void ReadPLY(const std::string& filename, std::vector<Eigen::Vector3d>& points,
              std::vector<Eigen::Vector3d>& normals,
              std::vector<Eigen::Vector3d>& colors) {

    points.clear();
    normals.clear();
    colors.clear();

    FILE* fp = fopen(filename.c_str(), "rb");
    if (fp == NULL) {
        // fprintf(stderr, "could not open file\n");
        // exit(1);
        throw std::runtime_error("could not open .ply file");
    }

    char buffer[100];
    char format[100];
    int size;
    char property_type[100];
    char property_name[100];
    int n_property = 0;
    int vertex_byte_offset[3];
    int byte_size = 0;
    bool is_double = false;
    bool normal_exists = false;
    int normal_byte_offset[3];
    bool color_exists = false;
    int color_byte_offset[3];

    fgets(buffer, 99, fp);
    // char str_ply[] = "ply\n";
    if (strcmp(buffer,"ply\n")!=0) {
        fprintf(stderr,"corrupted .ply file\n");
    }

    fgets(buffer, 99, fp);
    sscanf(buffer, "format %s", format);

    fgets(buffer, 99, fp);
    // char str_element_vertex[] = "element vertex";
    while (strncmp(buffer, "element vertex", 14) != 0) {
      fgets(buffer, 99, fp);
    }

    sscanf(buffer, "element vertex %d", &size);

    // char str_float[] = "float";
    // char str_double[] = "double";
    // char str_uchar[] = "uchar";
    fgets(buffer, 99, fp);
    while (sscanf(buffer, "property %s %s",
            property_type, property_name)==2) {
      n_property++;
      if (strcmp(property_name, "x")==0) {
          vertex_byte_offset[0] = byte_size;
          if (strcmp(property_type, "double")==0) {
              is_double = true;
          }
      }
      if (strcmp(property_name, "y")==0) {
          vertex_byte_offset[1] = byte_size;
      }
      if (strcmp(property_name, "z")==0) {
          vertex_byte_offset[2] = byte_size;
      }
      if (strcmp(property_name, "nx")==0) {
          normal_exists = true;
          normal_byte_offset[0] = byte_size;
      }
      if (strcmp(property_name, "ny")==0) {
          normal_byte_offset[1] = byte_size;
      }
      if (strcmp(property_name, "nz")==0) {
          normal_byte_offset[2] = byte_size;
      }
      if (strcmp(property_name, "red")==0) {
          color_exists = true;
          color_byte_offset[0] = byte_size;
      }
      if (strcmp(property_name, "green")==0) {
          color_byte_offset[1] = byte_size;
      }
      if (strcmp(property_name, "blue")==0) {
          color_byte_offset[2] = byte_size;
      }

      if (strcmp(property_type,"float")==0) {
          byte_size += sizeof(float);
      } else if (strcmp(property_type,"double")==0) {
          byte_size += sizeof(double);
      } else if (strcmp(property_type,"uchar")==0) {
          byte_size += sizeof(unsigned char);
      } else {
        // fprintf(stderr,"unknown property type\n");
        // exit(1);
        fclose(fp);
        throw std::runtime_error("unknown property type");
      }
      fgets(buffer, 99, fp);
    }

    // char str_end_header[] = "end_header\n";
    while(strcmp(buffer, "end_header\n")!=0) {
        fgets(buffer, 99, fp);
    }

    // char str_ascii[] = "ascii";
    // char str_binary[] = "binary_little_endian";
    if (strcmp(format,"binary_little_endian")==0) {
        points.resize(size);
        if (normal_exists) {
          normals.resize(size);
        }
        if (color_exists) {
          colors.resize(size);
        }
        if (is_double) {
            double x, y, z;
            double nx, ny, nz;
            unsigned char r, g, b;
            bool swap_endian = !is_little_endian();
            for (int i = 0; i < size; i++) {
              fread(buffer, 1, byte_size, fp);
              if (swap_endian) {
                  swap_bytes(buffer+vertex_byte_offset[0],sizeof(double));
                  swap_bytes(buffer+vertex_byte_offset[1],sizeof(double));
                  swap_bytes(buffer+vertex_byte_offset[2],sizeof(double));
              }
              memcpy(&x,buffer+vertex_byte_offset[0],sizeof(double));
              memcpy(&y,buffer+vertex_byte_offset[1],sizeof(double));
              memcpy(&z,buffer+vertex_byte_offset[2],sizeof(double));
              points[i][0] = x;
              points[i][1] = y;
              points[i][2] = z;
              if (normal_exists) {
                  if (swap_endian) {
                      swap_bytes(buffer+normal_byte_offset[0],sizeof(double));
                      swap_bytes(buffer+normal_byte_offset[1],sizeof(double));
                      swap_bytes(buffer+normal_byte_offset[2],sizeof(double));
                  }
                  memcpy(&nx,buffer+normal_byte_offset[0],sizeof(double));
                  memcpy(&ny,buffer+normal_byte_offset[1],sizeof(double));
                  memcpy(&nz,buffer+normal_byte_offset[2],sizeof(double));
                  normals[i][0] = nx;
                  normals[i][1] = ny;
                  normals[i][2] = nz;
              }
              if (color_exists) {
                  memcpy(&r,buffer+color_byte_offset[0],sizeof(unsigned char));
                  memcpy(&g,buffer+color_byte_offset[1],sizeof(unsigned char));
                  memcpy(&b,buffer+color_byte_offset[2],sizeof(unsigned char));
                  colors[i][0] = r / 255.;
                  colors[i][1] = g / 255.;
                  colors[i][2] = b / 255.;
              }
            }
        } else {
            float x, y, z;
            float nx, ny, nz;
            unsigned char r, g, b;
            bool swap_endian = !is_little_endian();
            for (int i = 0; i < size; i++) {
              fread(buffer, 1, byte_size, fp);
              if (swap_endian) {
                  swap_bytes(buffer+vertex_byte_offset[0],sizeof(float));
                  swap_bytes(buffer+vertex_byte_offset[1],sizeof(float));
                  swap_bytes(buffer+vertex_byte_offset[2],sizeof(float));
              }
              memcpy(&x,buffer+vertex_byte_offset[0],sizeof(float));
              memcpy(&y,buffer+vertex_byte_offset[1],sizeof(float));
              memcpy(&z,buffer+vertex_byte_offset[2],sizeof(float));
              points[i][0] = x;
              points[i][1] = y;
              points[i][2] = z;
              if (normal_exists) {
                  if (swap_endian) {
                      swap_bytes(buffer+normal_byte_offset[0],sizeof(float));
                      swap_bytes(buffer+normal_byte_offset[1],sizeof(float));
                      swap_bytes(buffer+normal_byte_offset[2],sizeof(float));
                  }
                  memcpy(&nx,buffer+normal_byte_offset[0],sizeof(float));
                  memcpy(&ny,buffer+normal_byte_offset[1],sizeof(float));
                  memcpy(&nz,buffer+normal_byte_offset[2],sizeof(float));
                  normals[i][0] = nx;
                  normals[i][1] = ny;
                  normals[i][2] = nz;
              }
              if (color_exists) {
                  memcpy(&r,buffer+color_byte_offset[0],sizeof(unsigned char));
                  memcpy(&g,buffer+color_byte_offset[1],sizeof(unsigned char));
                  memcpy(&b,buffer+color_byte_offset[2],sizeof(unsigned char));
                  colors[i][0] = r / 255.;
                  colors[i][1] = g / 255.;
                  colors[i][2] = b / 255.;
              }
            }
        }
    } else if (strcmp(format,"ascii")==0) {
        points.resize(3*size);
        for (int i = 0; i < size; i++) {
          fgets(buffer, 99, fp);
          sscanf(buffer, "%lf %lf %lf", &points[i][0], &points[i][1], &points[i][2]);
        }
    } else {
        // fprintf(stderr, "unknown format\n");
        // exit(1);
        throw std::runtime_error("unknown format");
    }

    fclose(fp);
    return;
}
