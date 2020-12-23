#include <stdio.h>
#include <iostream>
#include <json/value.h>




typedef struct RegistrationParam {
  // struct definition
  enum struct RegistrationType {
    ICP, RANSAC, FAST
  };
  // variable
  RegistrationType registration_type_;
  // constructor
  RegistrationType(RegistrationType type) : registration_type_(type) {}
} RegistrationParam;

typedef struct ICPParam : RegistrationParam {
  // struct definition
  enum struct ICPType {
    Point2Point, Point2Plane
  };
  // variable
  ICPType icp_type_;
  double relative_fitness_;
  double relative_rmse_;
  int max_iteration_;
  // constructor
  ICPParam() : RegistrationParam(RegistrationType::ICP),
    icp_type_(ICPType::Point2Point), relative_fitness_(1e-6),
    relative_rmse_(1e-6), max_iteration_(30) {}
  ICPParam(double relative_fitness, double relative_rmse, int max_iteration)
    : RegistrationParam(RegistrationType::ICP),
    icp_type_(ICPType::Point2Point), relative_fitness_(relative_fitness),
    relative_rmse_(relative_rmse), max_iteration_(max_iteration) {}
  ICPParam(double relative_fitness, double relative_rmse, int max_iteration)
    : RegistrationParam(RegistrationType::ICP),
    icp_type_(ICPType::Point2Point), relative_fitness_(relative_fitness),
    relative_rmse_(relative_rmse), max_iteration_(max_iteration) {}
} ICPParam;

typedef struct FastParam : RegistrationParam {
  // variable
  double relative_fitness_;
  double relative_rmse_;
  int max_iteration_;
  // constructor
  ICPParam() : RegistrationParam(RegistrationType::ICP),
    relative_fitness_(1e-6), relative_rmse_(1e-6), max_iteration_(30) {}
  ICPParam(double relative_fitness, double relative_rmse, int max_iteration)
    : RegistrationParam(RegistrationType::ICP), relative_fitness_(relative_fitness),
      relative_rmse_(relative_rmse), max_iteration_(max_iteration) {}
}
