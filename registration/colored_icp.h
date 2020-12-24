#pragma once

#include <Eigen/Core>
#include <memory>

#include "registration.h"
#include "robust_kernel.h"
#include "transformation_estimation.h"

class PointCloud;
class RegistrationResult;

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
