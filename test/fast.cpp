#include "io.h"
#include "pointcloud.h"
#include "utils.h"
#include "registration.h"
#include "fast_global_registration.h"
#include "feature.h"

#include <vector>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <iostream>

Eigen::Matrix4d read_rotation(std::string filename) {
	FILE* fp = fopen(filename.c_str(), "r");
	if (fp == NULL) {
		fprintf(stderr, "couldn't read txt file\n");
		exit(1);
	}

	Eigen::Matrix4d x;
	fscanf(fp,"%lf %lf %lf %lf \n %lf %lf %lf %lf \n %lf %lf %lf %lf \n %lf %lf %lf %lf \n",
	&x(0,0), &x(0,1), &x(0,2), &x(0,3), &x(1,0), &x(1,1), &x(1,2), &x(1,3),
	&x(2,0), &x(2,1), &x(2,2), &x(2,3), &x(3,0), &x(3,1), &x(3,2), &x(3,3));

	fclose(fp);
	return x;
}

int main(int argc, char** argv) {

	std::shared_ptr<PointCloud> source;
	std::string filename1(argv[1]);
	if (filename1.compare(filename1.size()-4,4,".stl")==0) {
		source = std::make_shared<PointCloud>(read_stl(filename1));
		write_pts(*source,filename1+".pts");
	} else if (filename1.compare(filename1.size()-4,4,".pts")==0) {
		source = std::make_shared<PointCloud>(read_pts(filename1));
	} else {
		exit(1);
	}

	std::cout << "Number of Points" << std::endl;
	std::cout << source->points_.size() << std::endl << std::endl;

	std::shared_ptr<PointCloud> target;

	if (argc > 2) {
		std::string filename2(argv[2]);
		if (filename2.compare(filename2.size()-4,4,".stl")==0) {
			target = std::make_shared<PointCloud>(read_stl(filename2));
			write_pts(*target,filename2+".pts");
		} else if (filename2.compare(filename2.size()-4,4,".pts")==0) {
			target = std::make_shared<PointCloud>(read_pts(filename2));
	 	} else if (filename2.compare(filename2.size()-4,4,".txt")==0) {
			Eigen::Matrix4d R = read_rotation(filename2);
			target = std::make_shared<PointCloud>(*source);
			target->Transform(R);
			write_pts(*target,filename1+"_randtsfm.pts");
		} else if (filename2.compare(0,6,"random")==0) {
			Eigen::Matrix4d R = generate_random_rotation(1, 1, 1, 1);
			target = std::make_shared<PointCloud>(*source);
			target->Transform(R);
			std::cout << "Generated Target Transformation" << std::endl;
			std::cout << R << std::endl << std::endl;
		} else {
			exit(1);
		}
	} else {
		Eigen::Matrix4d R = generate_random_rotation(1, 1, 1, 1);
		target = std::make_shared<PointCloud>(*source);
		target->Transform(R);
		std::cout << "Generated Target Transformation" << std::endl;
		std::cout << R << std::endl << std::endl;
	}

  auto source_d = source->VoxelDownSample(2.0);
  auto target_d = target->VoxelDownSample(2.0);
  source_d->EstimateNormals();
  target_d->EstimateNormals();

  // calculate feature
  std::shared_ptr<Feature> source_d_fpfh = ComputeFPFHFeature(*source_d);
  std::shared_ptr<Feature> target_d_fpfh = ComputeFPFHFeature(*target_d);

  std::cout << "starting Fast Global Registration" << std::endl;

  // fast global registration
  RegistrationResult fast_result = FastGlobalRegistration(
                                              *source_d,
                                              *target_d,
                                              *source_d_fpfh,
                                              *target_d_fpfh,
																							FastGlobalRegistrationOption());

  source->Transform(fast_result.transformation_);

  std::cout << std::endl << "Fast Transformation" << std::endl;
	std::cout << fast_result.transformation_ << std::endl << std::endl;
	std::cout << "Number of Correspondence Sets" << std::endl;
	std::cout << fast_result.correspondence_set_.size() << std::endl << std::endl;
	std::cout << "Fitness" << std::endl;
	std::cout << fast_result.fitness_ << std::endl << std::endl;
		std::cout << "Inlier RMSE" << std::endl;
	std::cout << fast_result.inlier_rmse_ << std::endl;
	if (argc > 3) {
		std::string filename3(argv[3]);
		write_pts(*source,filename3);
	} else {
		write_pts(*source,"fast_result.pts");
	}

  // auto source_d = source->VoxelDownSample(0.2);
  // auto target_d = target->VoxelDownSample(0.2);
  // source_d->EstimateNormals();
  // target_d->EstimateNormals();
  //
	// // ICP registration
	// RegistrationResult icp_result = RegistrationICP(
	// 									  *source_d,
	// 									  *target_d,
	// 									  0.2,
	// 									  true,
	// 									  Eigen::Matrix4d::Identity(),
	// 									  TransformationEstimationPointToPlane());
  //
	// source->Transform(icp_result.transformation_);
  //
  // std::cout << std::endl << "ICP" << std::endl;
	// std::cout << "Source Rotation" << std::endl;
	// std::cout << icp_result.transformation_ << std::endl << std::endl;
	// std::cout << "Number of Correspondence Sets" << std::endl;
	// std::cout << icp_result.correspondence_set_.size() << std::endl << std::endl;
	// std::cout << "Fitness" << std::endl;
	// std::cout << icp_result.fitness_ << std::endl << std::endl;
	// 	std::cout << "Inlier RMSE" << std::endl;
	// std::cout << icp_result.inlier_rmse_ << std::endl;
	// write_pts(*source,"icp_result.pts");

  // std::cout << pcd.points_.size() << std::endl;
  // std::cout << dpcd->points_.size() << std::endl;
}
