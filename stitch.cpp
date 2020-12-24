#include "registration/io.h"
#include "registration/pointcloud.h"
#include "registration/utils.h"
#include "registration/registration.h"
#include "registration/fast_global_registration.h"
#include "registration/feature.h"
#include "registration/colored_icp.h"

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
    std::make_shared<PointCloud>(config["source"]["path"].asString());

  std::shared_ptr<PointCloud> source_d;
  if (config["source"]["downsample"].asBool()) {
      source_d = source->VoxelDownSample(config["source"]["voxel_size"].asFloat());
  } else {
      source_d = source;
  }

  std::shared_ptr<PointCloud> target =
    std::make_shared<PointCloud>(config["target"]["path"].asString());

  std::shared_ptr<PointCloud> target_d;
  if (config["target"]["downsample"].asBool()) {
      target_d = target->VoxelDownSample(config["target"]["voxel_size"].asFloat());
  } else {
      target_d = target;
  }

  std::string output_path = config["output_path"].asString();

  std::string reg_type = config["registration_type"].asString();

  if (reg_type == "ICP") {

      std::string icp_type = config["registration_parameters"]["icp"]["estimation_type"].asString();

      ICPConvergenceCriteria icp_param(config["registration_parameters"]["icp"]["relative_fitness"].asDouble(),
                                       config["registration_parameters"]["icp"]["relative_rmse"].asDouble(),
                                       config["registration_parameters"]["icp"]["max_iteration"].asInt());

      if ( ( (icp_type == "PointToPlane") || (icp_type == "Colored") ) && !target_d->HasNormals() ) {
          printf("Estimating normals for target\n");
          target_d->EstimateNormals();
      }

      RegistrationResult icp_result;

      if (icp_type != "Colored") {

          std::shared_ptr<TransformationEstimation> estimation_type;
          if (icp_type == "PointToPoint") {
              estimation_type = std::make_shared<TransformationEstimationPointToPoint>();
              printf("Performing Point-to-Point ICP Registration...\n");
          } else if (icp_type == "PointToPlane") {
              estimation_type = std::make_shared<TransformationEstimationPointToPlane>();
              printf("Performing Point-to-Plane ICP Registration...\n");
          } else {
              printf("Could not interpret ICP Registration Estimation Type\n");
              exit(1);
          }

          icp_result = RegistrationICP(*source_d, *target_d,
                config["registration_parameters"]["icp"]["max_correspondence_distance"].asDouble(),
                true, Eigen::Matrix4d::Identity(), *estimation_type,
                icp_param);

      } else {

          double lambda_geometric = config["registration_parameters"]["icp"]["lambda_geometric"].asDouble();
          std::string kernel_type = config["registration_parameters"]["icp"]["robust_kernel"].asString();
          std::shared_ptr<RobustKernel> kernel;
          if (kernel_type == "L2") {
              kernel = std::make_shared<L2Loss>();
          } else if (kernel_type == "L1") {
              kernel = std::make_shared<L1Loss>();
          } else {
              printf("Could not interpret robust kernel type\n");
              exit(1);
          }

          TransformationEstimationForColoredICP estimation(lambda_geometric, kernel);
          printf("Performing Colored ICP Registration...\n");
          icp_result = RegistrationColoredICP(*source_d, *target_d,
                config["registration_parameters"]["icp"]["max_correspondence_distance"].asDouble(),
                true, Eigen::Matrix4d::Identity(), estimation,
                icp_param);

      }

      source->Transform(icp_result.transformation_);
      WritePCD(*source, output_path);
      std::cout << std::endl << "ICP Transformation" << std::endl;
      std::cout << icp_result.transformation_ << std::endl << std::endl;
      std::cout << "Number of Correspondence Sets" << std::endl;
      std::cout << icp_result.correspondence_set_.size() << std::endl << std::endl;
      std::cout << "Fitness" << std::endl;
      std::cout << icp_result.fitness_ << std::endl << std::endl;
      std::cout << "Inlier RMSE" << std::endl;
      std::cout << icp_result.inlier_rmse_ << std::endl;

  } else if (reg_type == "RANSAC") {

      std::string ransac_base = config["registration_parameters"]["ransac"]["based_on"].asString();
      std::string ransac_type = config["registration_parameters"]["ransac"]["estimation_type"].asString();
      RANSACConvergenceCriteria ransac_param(config["registration_parameters"]["ransac"]["max_iteration"].asInt(),
                                              config["registration_parameters"]["ransac"]["confidence"].asDouble());

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
              *source_d_fpfh, *target_d_fpfh, config["registration_parameters"]["ransac"]["mutual_filter"].asBool(),
              config["registration_parameters"]["ransac"]["max_correspondence_distance"].asDouble(),
              *estimation_type,
              config["registration_parameters"]["ransac"]["ransac_n"].asInt(), {},
              ransac_param);

      } else {
          printf("Could not interpret RANSAC Registration Base\n");
          exit(1);
      }

      source->Transform(ransac_result.transformation_);
      WritePCD(*source, output_path);
      std::cout << std::endl << "RANSAC Registration Result" << std::endl;
      std::cout << ransac_result.transformation_ << std::endl << std::endl;
      std::cout << "Number of Correspondence Sets" << std::endl;
      std::cout << ransac_result.correspondence_set_.size() << std::endl << std::endl;
      std::cout << "Fitness" << std::endl;
      std::cout << ransac_result.fitness_ << std::endl << std::endl;
      std::cout << "Inlier RMSE" << std::endl;
      std::cout << ransac_result.inlier_rmse_ << std::endl;

  } else if (reg_type == "Fast") {

      FastGlobalRegistrationOption fast_param(config["registration_parameters"]["fast"]["division_factor"].asDouble(),
                                              config["registration_parameters"]["fast"]["use_absolute_scale"].asBool(),
                                              config["registration_parameters"]["fast"]["decrease_mu"].asBool(),
                                              config["registration_parameters"]["fast"]["max_correspondence_distance"].asDouble(),
                                              config["registration_parameters"]["fast"]["iteration_number"].asInt(),
                                              config["registration_parameters"]["fast"]["tuple_scale"].asDouble(),
                                              config["registration_parameters"]["fast"]["max_tuple_count"].asInt());
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
      WritePCD(*source, output_path);
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
