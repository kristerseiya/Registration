#include "registration/io.h"
#include "registration/pointcloud.h"
#include "registration/utils.h"
#include "registration/registration.h"
#include "registration/fast_global_registration.h"
#include "registration/feature.h"

#include <iostream>
#include <fstream>
#include <json/value.h>
#include <json/reader.h>
#include <string>
#include <memory>
#include <Eigen/Core>

int main(int argc, char** argv) {
  std::fstream config_file(argv[1]);
  Json::Value config;
  config_file >> config;

  std::shared_ptr<PointCloud> source =
    std::make_shared<PointCloud>(ReadPointsFromFile(config["source"]["path"].asString()));

  std::shared_ptr<PointCloud> source_d;
  if (config["source"]["downsample"].asBool()) {
      source_d = source->VoxelDownSample(config["source"]["voxel_size"].asFloat());
  } else {
      source_d = source;
  }

  std::shared_ptr<PointCloud> target =
    std::make_shared<PointCloud>(ReadPointsFromFile(config["target"]["path"].asString()));

  std::shared_ptr<PointCloud> target_d;
  if (config["target"]["downsample"].asBool()) {
      target_d = target->VoxelDownSample(config["target"]["voxel_size"].asFloat());
  } else {
      target_d = target;
  }

  std::string output_path = config["output_path"].asString();

  std::string reg_type = config["registration_type"].asString();

  if (reg_type == "ICP") {

      std::string icp_type = config["registration_parameters"][0]["estimation_type"].asString();

      ICPConvergenceCriteria icp_param(config["registration_parameters"][0]["relative_fitness"].asDouble(),
                                       config["registration_parameters"][0]["relative_rmse"].asDouble(),
                                       config["registration_parameters"][0]["max_iteration"].asInt());

      if ( (icp_type == "PointToPlane") && !target_d->HasNormals() ) {
          printf("Estimating normals for target\n");
          target_d->EstimateNormals();
      }

      TransformationEstimation* estimation_type;
      TransformationEstimationPointToPoint point2point_estimation;
      TransformationEstimationPointToPlane point2plane_estimation;
      if (icp_type == "PointToPoint") {
          estimation_type = &point2point_estimation;
      } else if (icp_type == "PointToPlane") {
          estimation_type = &point2plane_estimation;
      } else {
          printf("Could not interpret ICP Registration Estimation Type\n");
          exit(1);
      }

      printf("Performing ICP Registration...\n");
      RegistrationResult icp_result = RegistrationICP(*source_d, *target_d,
            config["registration_parameters"][0]["max_correspondence_distance"].asDouble(),
            true, Eigen::Matrix4d::Identity(), TransformationEstimationPointToPoint(),
            icp_param);

      source->Transform(icp_result.transformation_);
      write_pcd(*source, output_path);
      std::cout << std::endl << "ICP Transformation" << std::endl;
      std::cout << icp_result.transformation_ << std::endl << std::endl;
      std::cout << "Number of Correspondence Sets" << std::endl;
      std::cout << icp_result.correspondence_set_.size() << std::endl << std::endl;
      std::cout << "Fitness" << std::endl;
      std::cout << icp_result.fitness_ << std::endl << std::endl;
      std::cout << "Inlier RMSE" << std::endl;
      std::cout << icp_result.inlier_rmse_ << std::endl;

  } else if (reg_type == "RANSAC") {

      std::string ransac_base = config["registration_parameters"][1]["based_on"].asString();
      std::string ransac_type = config["registration_parameters"][1]["estimation_type"].asString();
      RANSACConvergenceCriteria ransac_param(config["registration_parameters"][1]["max_iteration"].asInt(),
                                              config["registration_parameters"][1]["confidence"].asDouble());

      if ( (ransac_type == "PointToPlane") && !target_d->HasNormals() ) {
          target_d->EstimateNormals();
          printf("Estimating normals for target\n");
      }

      TransformationEstimation* estimation_type;
      TransformationEstimationPointToPoint point2point_estimation;
      TransformationEstimationPointToPlane point2plane_estimation;
      if (ransac_type == "PointToPoint") {
          estimation_type = &point2point_estimation;
      } else if (ransac_type == "PointToPlane") {
          estimation_type = &point2plane_estimation;
      } else {
          printf("Could not interpret RANSAC Registration Estimation Type\n");
          exit(1);
      }

      RegistrationResult ransac_result;
      if (ransac_base == "feature") {
          if (!source_d->HasNormals()) {
            printf("Estimating normals for source\n");
            source_d->EstimateNormals();
          }
          if (!target_d->HasNormals()) {
            printf("Estimating normals for target\n");
            target_d->EstimateNormals();
          }
          printf("Computing FPFH Feature for source\n");
          std::shared_ptr<Feature> source_d_fpfh = ComputeFPFHFeature(*source_d);
          printf("Computing FPFH Feature for target\n");
          std::shared_ptr<Feature> target_d_fpfh = ComputeFPFHFeature(*target_d);

          printf("Performing RANSAC Registration Based on FPFH Features...\n");
          ransac_result = RegistrationRANSACBasedOnFeatureMatching(*source_d, *target_d,
              *source_d_fpfh, *target_d_fpfh, config["registration_parameters"][1]["mutual_filter"].asBool(),
              config["registration_parameters"][1]["max_correspondence_distance"].asDouble(),
              *estimation_type,
              config["registration_parameters"][1]["ransac_n"].asInt(), {},
              ransac_param);

      } else {
          printf("Could not interpret RANSAC Registration Base\n");
          exit(1);
      }

      source->Transform(ransac_result.transformation_);
      write_pcd(*source, output_path);
      std::cout << std::endl << "RANSAC Registration Result" << std::endl;
      std::cout << ransac_result.transformation_ << std::endl << std::endl;
      std::cout << "Number of Correspondence Sets" << std::endl;
      std::cout << ransac_result.correspondence_set_.size() << std::endl << std::endl;
      std::cout << "Fitness" << std::endl;
      std::cout << ransac_result.fitness_ << std::endl << std::endl;
      std::cout << "Inlier RMSE" << std::endl;
      std::cout << ransac_result.inlier_rmse_ << std::endl;

  } else if (reg_type == "Fast") {

      FastGlobalRegistrationOption fast_param(config["registration_parameters"][2]["division_factor"].asDouble(),
                                              config["registration_parameters"][2]["use_absolute_scale"].asBool(),
                                              config["registration_parameters"][2]["decrease_mu"].asBool(),
                                              config["registration_parameters"][2]["max_correspondence_distance"].asDouble(),
                                              config["registration_parameters"][2]["iteration_number"].asInt(),
                                              config["registration_parameters"][2]["tuple_scale"].asDouble(),
                                              config["registration_parameters"][2]["max_tuple_count"].asInt());
      if (!source_d->HasNormals()) {
        printf("Estimating normals for source\n");
        source_d->EstimateNormals();
      }
      if (!target_d->HasNormals()) {
        printf("Estimating normals for target\n");
        target_d->EstimateNormals();
      }
      printf("Computing FPFH Feature for source\n");
      std::shared_ptr<Feature> source_d_fpfh = ComputeFPFHFeature(*source_d);
      printf("Computing FPFH Feature for target\n");
      std::shared_ptr<Feature> target_d_fpfh = ComputeFPFHFeature(*target_d);

      printf("Performing Fast Global Registration...\n");
      RegistrationResult fast_result = FastGlobalRegistration(*source_d, *target_d,
          *source_d_fpfh, *target_d_fpfh, fast_param);

      source->Transform(fast_result.transformation_);
      write_pcd(*source, output_path);
      std::cout << std::endl << "Fast Global Registration Result" << std::endl;
      std::cout << fast_result.transformation_ << std::endl << std::endl;
      std::cout << "Number of Correspondence Sets" << std::endl;
      std::cout << fast_result.correspondence_set_.size() << std::endl << std::endl;
      std::cout << "Fitness" << std::endl;
      std::cout << fast_result.fitness_ << std::endl << std::endl;
      std::cout << "Inlier RMSE" << std::endl;
      std::cout << fast_result.inlier_rmse_ << std::endl;

  } else {

      printf("Could not interpret Registration Type\n");
      exit(1);
  }

  exit(0);
}
