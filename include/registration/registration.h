// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>
#include <tuple>
#include <vector>
#include <Eigen/Geometry>
#include <memory>

#include "correspondence_checker.h"
#include "transformation_estimation.h"
#include "eigen.h"
#include "robust_kernel.h"

class PointCloud;

class Feature;

/// \class ICPConvergenceCriteria
///
/// \brief Class that defines the convergence criteria of ICP.
///
/// ICP algorithm stops if the relative change of fitness and rmse hit
/// \p relative_fitness_ and \p relative_rmse_ individually, or the iteration
/// number exceeds \p max_iteration_.
class ICPConvergenceCriteria {
public:
    /// \brief Parameterized Constructor.
    ///
    /// \param relative_fitness If relative change (difference) of fitness score
    /// is lower than relative_fitness, the iteration stops. \param
    /// relative_rmse If relative change (difference) of inliner RMSE score is
    /// lower than relative_rmse, the iteration stops. \param max_iteration
    /// Maximum iteration before iteration stops.
    ICPConvergenceCriteria(double relative_fitness = 1e-6,
                           double relative_rmse = 1e-6,
                           int max_iteration = 30)
        : relative_fitness_(relative_fitness),
          relative_rmse_(relative_rmse),
          max_iteration_(max_iteration) {}
    ~ICPConvergenceCriteria() {}

public:
    /// If relative change (difference) of fitness score is lower than
    /// `relative_fitness`, the iteration stops.
    double relative_fitness_;
    /// If relative change (difference) of inliner RMSE score is lower than
    /// `relative_rmse`, the iteration stops.
    double relative_rmse_;
    /// Maximum iteration before iteration stops.
    int max_iteration_;
};

/// \class RANSACConvergenceCriteria
///
/// \brief Class that defines the convergence criteria of RANSAC.
///
/// RANSAC algorithm stops if the iteration number hits max_iteration_, or the
/// validation has been run for max_validation_ times.
/// Note that the validation is the most computational expensive operator in an
/// iteration. Most iterations do not do full validation. It is crucial to
/// control max_validation_ so that the computation time is acceptable.
class RANSACConvergenceCriteria {
public:
    /// \brief Parameterized Constructor.
    ///
    /// \param max_iteration Maximum iteration before iteration stops.
    /// \param confidence Desired probability of success. Used for estimating
    /// early termination by k = log(1 - confidence)/log(1 -
    /// inlier_ratio^{ransac_n}).
    RANSACConvergenceCriteria(int max_iteration = 100000,
                              double confidence = 0.999)
        : max_iteration_(max_iteration), confidence_(confidence) {}

    ~RANSACConvergenceCriteria() {}

public:
    /// Maximum iteration before iteration stops.
    int max_iteration_;
    /// Desired probability of success.
    double confidence_;
};

/// \class RegistrationResult
///
/// Class that contains the registration results.
class RegistrationResult {
public:
    /// \brief Parameterized Constructor.
    ///
    /// \param transformation The estimated transformation matrix.
    RegistrationResult(
            const Eigen::Matrix4d &transformation = Eigen::Matrix4d::Identity())
        : transformation_(transformation), inlier_rmse_(0.0), fitness_(0.0) {}
    ~RegistrationResult() {}
    bool IsBetterRANSACThan(const RegistrationResult &other) const {
        return fitness_ > other.fitness_ || (fitness_ == other.fitness_ &&
                                             inlier_rmse_ < other.inlier_rmse_);
    }

public:
    /// The estimated transformation matrix.
    Eigen::Matrix4d_u transformation_;
    /// Correspondence set between source and target point cloud.
    CorrespondenceSet correspondence_set_;
    /// RMSE of all inlier correspondences. Lower is better.
    double inlier_rmse_;
    /// For ICP: the overlapping area (# of inlier correspondences / # of points
    /// in target). Higher is better.
    /// For RANSAC: inlier ratio (# of inlier correspondences / # of
    /// all correspondences)
    double fitness_;
};

/// \brief Function for evaluating registration between point clouds.
///
/// \param source The source point cloud.
/// \param target The target point cloud.
/// \param max_correspondence_distance Maximum correspondence points-pair
/// distance. \param transformation The 4x4 transformation matrix to transform
/// source to target. Default value: array([[1., 0., 0., 0.], [0., 1., 0., 0.],
/// [0., 0., 1., 0.], [0., 0., 0., 1.]]).
RegistrationResult EvaluateRegistration(
        const PointCloud &source,
        const PointCloud &target,
        double max_correspondence_distance,
        const Eigen::Matrix4d &transformation = Eigen::Matrix4d::Identity());

/// \brief Functions for ICP registration.
///
/// \param source The source point cloud.
/// \param target The target point cloud.
/// \param max_correspondence_distance Maximum correspondence points-pair
/// distance. \param init Initial transformation estimation.
///  Default value: array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.],
///  [0., 0., 0., 1.]])
/// \param estimation Estimation method.
/// \param criteria Convergence criteria.
RegistrationResult RegistrationICP(
        const PointCloud &source,
        const PointCloud &target,
        double max_correspondence_distance,
        bool verbose = true,
        const Eigen::Matrix4d &init = Eigen::Matrix4d::Identity(),
        const TransformationEstimation &estimation =
                TransformationEstimationPointToPoint(false),
        const ICPConvergenceCriteria &criteria = ICPConvergenceCriteria());

/// \brief Function for global RANSAC registration based on a given set of
/// correspondences.
///
/// \param source The source point cloud.
/// \param target The target point cloud.
/// \param corres Correspondence indices between source and target point clouds.
/// \param max_correspondence_distance Maximum correspondence points-pair
/// distance.
/// \param estimation Estimation method.
/// \param ransac_n Fit ransac with `ransac_n` correspondences.
/// \param checkers Correspondence checker.
/// \param criteria Convergence criteria.
RegistrationResult RegistrationRANSACBasedOnCorrespondence(
        const PointCloud &source,
        const PointCloud &target,
        const CorrespondenceSet &corres,
        double max_correspondence_distance,
        const TransformationEstimation &estimation =
                TransformationEstimationPointToPoint(false),
        int ransac_n = 3,
        const std::vector<std::reference_wrapper<const CorrespondenceChecker>>
                &checkers = {},
        const RANSACConvergenceCriteria &criteria =
                RANSACConvergenceCriteria());

/// \brief Function for global RANSAC registration based on feature matching.
///
/// \param source The source point cloud.
/// \param target The target point cloud.
/// \param source_feature Source point cloud feature.
/// \param target_feature Target point cloud feature.
/// \param mutual_filter Enables mutual filter such that the correspondence of
/// the source point's correspondence is itself.
/// \param max_correspondence_distance Maximum correspondence points-pair
/// distance.
/// \param ransac_n Fit ransac with `ransac_n` correspondences.
/// \param checkers Correspondence checker.
/// \param criteria Convergence criteria.
RegistrationResult RegistrationRANSACBasedOnFeatureMatching(
        const PointCloud &source,
        const PointCloud &target,
        const Feature &source_feature,
        const Feature &target_feature,
        bool mutual_filter,
        double max_correspondence_distance,
        const TransformationEstimation &estimation =
                TransformationEstimationPointToPoint(false),
        int ransac_n = 3,
        const std::vector<std::reference_wrapper<const CorrespondenceChecker>>
                &checkers = {},
        const RANSACConvergenceCriteria &criteria =
                RANSACConvergenceCriteria());

/// \param source The source point cloud.
/// \param target The target point cloud.
/// \param max_correspondence_distance Maximum correspondence points-pair
/// distance. \param transformation The 4x4 transformation matrix to transform
/// `source` to `target`.
Eigen::Matrix6d GetInformationMatrixFromPointClouds(
        const PointCloud &source,
        const PointCloud &target,
        double max_correspondence_distance,
        const Eigen::Matrix4d &transformation);

        /// \class FastGlobalRegistrationOption
        ///
        /// \brief Options for FastGlobalRegistration.
        class FastGlobalRegistrationOption {
        public:
            /// \brief Parameterized Constructor.
            ///
            /// \param division_factor Division factor used for graduated non-convexity.
            /// \param use_absolute_scale Measure distance in absolute scale (1) or in
            /// scale relative to the diameter of the model (0).
            /// \param decrease_mu Set
            /// to `true` to decrease scale mu by division_factor for graduated
            /// non-convexity.
            /// \param maximum_correspondence_distance Maximum
            /// correspondence distance (also see comment of USE_ABSOLUTE_SCALE).
            /// \param iteration_number Maximum number of iterations.
            /// \param tuple_scale Similarity measure used for tuples of feature points.
            /// \param maximum_tuple_count Maximum numer of tuples.
            FastGlobalRegistrationOption(double division_factor = 1.4,
                                         bool use_absolute_scale = false,
                                         bool decrease_mu = true,
                                         double maximum_correspondence_distance = 0.025,
                                         int iteration_number = 64,
                                         double tuple_scale = 0.95,
                                         int maximum_tuple_count = 1000)
                : division_factor_(division_factor),
                  use_absolute_scale_(use_absolute_scale),
                  decrease_mu_(decrease_mu),
                  maximum_correspondence_distance_(maximum_correspondence_distance),
                  iteration_number_(iteration_number),
                  tuple_scale_(tuple_scale),
                  maximum_tuple_count_(maximum_tuple_count) {}
            ~FastGlobalRegistrationOption() {}

        public:
            /// Division factor used for graduated non-convexity.
            double division_factor_;
            /// Measure distance in absolute scale (1) or in scale relative to the
            /// diameter of the model (0).
            bool use_absolute_scale_;
            /// Set to `true` to decrease scale mu by division_factor for graduated
            /// non-convexity.
            bool decrease_mu_;
            /// Maximum correspondence distance (also see comment of
            /// USE_ABSOLUTE_SCALE).
            double maximum_correspondence_distance_;
            /// Maximum number of iterations.
            int iteration_number_;
            /// Similarity measure used for tuples of feature points.
            double tuple_scale_;
            /// Maximum number of tuples..
            int maximum_tuple_count_;
        };

        RegistrationResult FastGlobalRegistration(
                const PointCloud &source,
                const PointCloud &target,
                const Feature &source_feature,
                const Feature &target_feature,
                const FastGlobalRegistrationOption &option =
                        FastGlobalRegistrationOption());

                        class TransformationEstimationForColoredICP : public TransformationEstimation {
                        public:
                            ~TransformationEstimationForColoredICP() override{};

                            TransformationEstimationType GetTransformationEstimationType()
                                    const override {
                                return type_;
                            };
                            explicit TransformationEstimationForColoredICP(
                                    double lambda_geometric = 0.968,
                                    std::shared_ptr<RobustKernel> kernel = std::make_shared<L2Loss>())
                                : lambda_geometric_(lambda_geometric), kernel_(std::move(kernel)) {
                                if (lambda_geometric_ < 0 || lambda_geometric_ > 1.0) {
                                    lambda_geometric_ = 0.968;
                                }
                            }

                        public:
                            double ComputeRMSE(const PointCloud &source,
                                               const PointCloud &target,
                                               const CorrespondenceSet &corres) const override;
                            Eigen::Matrix4d ComputeTransformation(
                                    const PointCloud &source,
                                    const PointCloud &target,
                                    const CorrespondenceSet &corres) const override;

                        public:
                            double lambda_geometric_ = 0.968;
                            /// shared_ptr to an Abstract RobustKernel that could mutate at runtime.
                            std::shared_ptr<RobustKernel> kernel_ = std::make_shared<L2Loss>();

                        private:
                            const TransformationEstimationType type_ =
                                    TransformationEstimationType::ColoredICP;
                        };

                        /// \brief Function for Colored ICP registration.
                        ///
                        /// This is implementation of following paper
                        /// J. Park, Q.-Y. Zhou, V. Koltun,
                        /// Colored Point Cloud Registration Revisited, ICCV 2017.
                        ///
                        /// \param source The source point cloud.
                        /// \param target The target point cloud.
                        /// \param max_distance Maximum correspondence points-pair distance.
                        /// \param init Initial transformation estimation.
                        /// Default value: array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.],
                        /// [0., 0., 0., 1.]]). \param criteria  Convergence criteria. \param
                        /// \param estimation TransformationEstimationForColoredICP method. Can only
                        /// change the lambda_geometric value and the robust kernel used in the
                        /// optimization
                        RegistrationResult RegistrationColoredICP(
                                const PointCloud &source,
                                const PointCloud &target,
                                double max_distance,
                                bool verbose = true,
                                const Eigen::Matrix4d &init = Eigen::Matrix4d::Identity(),
                                const TransformationEstimationForColoredICP &estimation =
                                        TransformationEstimationForColoredICP(),
                                const ICPConvergenceCriteria &criteria = ICPConvergenceCriteria());
