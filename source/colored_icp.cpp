// #include "colored_icp.h"

#include <Eigen/Dense>
#include <iostream>
#include <stdio.h>

#include "kdtree_flann.h"
// #include "kdtree_search_param.h"
#include "pointcloud.h"
#include "registration.h"
#include "eigen.h"

namespace {

class PointCloudForColoredICP : public PointCloud {
public:
    std::vector<Eigen::Vector3d> color_gradient_;
};

std::shared_ptr<PointCloudForColoredICP> InitializePointCloudForColoredICP(
        const PointCloud &target,
        const KDTreeSearchParamHybrid &search_param) {
    printf("InitializePointCloudForColoredICP\n");

    KDTreeFlann tree;
    tree.SetGeometry(target);

    auto output = std::make_shared<PointCloudForColoredICP>();
    output->colors_ = target.colors_;
    output->normals_ = target.normals_;
    output->points_ = target.points_;
    size_t n_points = output->points_.size();
    output->color_gradient_.resize(n_points, Eigen::Vector3d::Zero());

    for (size_t k = 0; k < n_points; k++) {
        const Eigen::Vector3d &vt = output->points_[k];
        const Eigen::Vector3d &nt = output->normals_[k];
        double it = (output->colors_[k](0) + output->colors_[k](1) +
                     output->colors_[k](2)) /
                    3.0;

        std::vector<int> point_idx;
        std::vector<double> point_squared_distance;

        if (tree.SearchHybrid(vt, search_param.radius_, search_param.max_nn_,
                              point_idx, point_squared_distance) >= 4) {

            // approximate image gradient of vt's tangential plane
            size_t nn = point_idx.size();
            Eigen::MatrixXd A(nn, 3);
            Eigen::MatrixXd b(nn, 1);
            A.setZero();
            b.setZero();

            for (size_t i = 1; i < nn; i++) {
                int P_adj_idx = point_idx[i];
                Eigen::Vector3d vt_adj = output->points_[P_adj_idx];
                Eigen::Vector3d vt_proj = vt_adj - (vt_adj - vt).dot(nt) * nt;
                double it_adj = (output->colors_[P_adj_idx](0) +
                                 output->colors_[P_adj_idx](1) +
                                 output->colors_[P_adj_idx](2)) /
                                3.0;
                A(i - 1, 0) = (vt_proj(0) - vt(0));
                A(i - 1, 1) = (vt_proj(1) - vt(1));
                A(i - 1, 2) = (vt_proj(2) - vt(2));
                b(i - 1, 0) = (it_adj - it);
            }

            // adds orthogonal constraint
            A(nn - 1, 0) = (nn - 1) * nt(0);
            A(nn - 1, 1) = (nn - 1) * nt(1);
            A(nn - 1, 2) = (nn - 1) * nt(2);
            b(nn - 1, 0) = 0;
            // solving linear equation
            bool is_success = false;
            Eigen::MatrixXd x;
            std::tie(is_success, x) = SolveLinearSystemPSD(
                    A.transpose() * A, A.transpose() * b);
            if (is_success) {
                output->color_gradient_[k] = x;
            }

        }
    }
    return output;
}

}  // namespace

Eigen::Matrix4d TransformationEstimationForColoredICP::ComputeTransformation(
        const PointCloud &source,
        const PointCloud &target,
        const CorrespondenceSet &corres) const {
    if (corres.empty() || !target.HasNormals() || !target.HasColors() ||
        !source.HasColors()) {
        return Eigen::Matrix4d::Identity();
    }

    double sqrt_lambda_geometric = sqrt(lambda_geometric_);
    double lambda_photometric = 1.0 - lambda_geometric_;
    double sqrt_lambda_photometric = sqrt(lambda_photometric);

    const auto &target_c = (const PointCloudForColoredICP &)target;

    auto compute_jacobian_and_residual =
            [&](int i,
                std::vector<Eigen::Vector6d, Vector6d_allocator> &J_r,
                std::vector<double> &r, std::vector<double> &w) {
                size_t cs = corres[i][0];
                size_t ct = corres[i][1];
                const Eigen::Vector3d &vs = source.points_[cs];
                const Eigen::Vector3d &vt = target.points_[ct];
                const Eigen::Vector3d &nt = target.normals_[ct];

                J_r.resize(2);
                r.resize(2);
                w.resize(2);

                J_r[0].block<3, 1>(0, 0) = sqrt_lambda_geometric * vs.cross(nt);
                J_r[0].block<3, 1>(3, 0) = sqrt_lambda_geometric * nt;
                r[0] = sqrt_lambda_geometric * (vs - vt).dot(nt);
                w[0] = kernel_->Weight(r[0]);

                // project vs into vt's tangential plane
                Eigen::Vector3d vs_proj = vs - (vs - vt).dot(nt) * nt;
                double is = (source.colors_[cs](0) + source.colors_[cs](1) +
                             source.colors_[cs](2)) /
                            3.0;
                double it = (target.colors_[ct](0) + target.colors_[ct](1) +
                             target.colors_[ct](2)) /
                            3.0;
                const Eigen::Vector3d &dit = target_c.color_gradient_[ct];
                double is0_proj = (dit.dot(vs_proj - vt)) + it;

                const Eigen::Matrix3d M =
                        (Eigen::Matrix3d() << 1.0 - nt(0) * nt(0),
                         -nt(0) * nt(1), -nt(0) * nt(2), -nt(0) * nt(1),
                         1.0 - nt(1) * nt(1), -nt(1) * nt(2), -nt(0) * nt(2),
                         -nt(1) * nt(2), 1.0 - nt(2) * nt(2))
                                .finished();

                const Eigen::Vector3d &ditM = -dit.transpose() * M;
                J_r[1].block<3, 1>(0, 0) =
                        sqrt_lambda_photometric * vs.cross(ditM);
                J_r[1].block<3, 1>(3, 0) = sqrt_lambda_photometric * ditM;
                r[1] = sqrt_lambda_photometric * (is - is0_proj);
                w[1] = kernel_->Weight(r[1]);
            };

    Eigen::Matrix6d JTJ;
    Eigen::Vector6d JTr;
    double r2;
    std::tie(JTJ, JTr, r2) =
            ComputeJTJandJTr<Eigen::Matrix6d, Eigen::Vector6d>(
                    compute_jacobian_and_residual, (int)corres.size());

    bool is_success;
    Eigen::Matrix4d extrinsic;
    std::tie(is_success, extrinsic) =
            SolveJacobianSystemAndObtainExtrinsicMatrix(JTJ, JTr);

    return is_success ? extrinsic : Eigen::Matrix4d::Identity();
}

double TransformationEstimationForColoredICP::ComputeRMSE(
        const PointCloud &source,
        const PointCloud &target,
        const CorrespondenceSet &corres) const {
    double sqrt_lambda_geometric = sqrt(lambda_geometric_);
    double lambda_photometric = 1.0 - lambda_geometric_;
    double sqrt_lambda_photometric = sqrt(lambda_photometric);
    const auto &target_c = (const PointCloudForColoredICP &)target;

    double residual = 0.0;
    for (size_t i = 0; i < corres.size(); i++) {
        size_t cs = corres[i][0];
        size_t ct = corres[i][1];
        const Eigen::Vector3d &vs = source.points_[cs];
        const Eigen::Vector3d &vt = target.points_[ct];
        const Eigen::Vector3d &nt = target.normals_[ct];
        Eigen::Vector3d vs_proj = vs - (vs - vt).dot(nt) * nt;
        double is = (source.colors_[cs](0) + source.colors_[cs](1) +
                     source.colors_[cs](2)) /
                    3.0;
        double it = (target.colors_[ct](0) + target.colors_[ct](1) +
                     target.colors_[ct](2)) /
                    3.0;
        const Eigen::Vector3d &dit = target_c.color_gradient_[ct];
        double is0_proj = (dit.dot(vs_proj - vt)) + it;
        double residual_geometric = sqrt_lambda_geometric * (vs - vt).dot(nt);
        double residual_photometric = sqrt_lambda_photometric * (is - is0_proj);
        residual += residual_geometric * residual_geometric +
                    residual_photometric * residual_photometric;
    }
    return residual;
};

RegistrationResult RegistrationColoredICP(
        const PointCloud &source,
        const PointCloud &target,
        double max_distance,
        bool verbose,
        const Eigen::Matrix4d &init /* = Eigen::Matrix4d::Identity()*/,
        const TransformationEstimationForColoredICP &estimation
        /*TransformationEstimationForColoredICP()*/,
        const ICPConvergenceCriteria
                &criteria /* = ICPConvergenceCriteria()*/) {
    auto target_c = InitializePointCloudForColoredICP(
            target, KDTreeSearchParamHybrid(max_distance * 2.0, 30));
    return RegistrationICP(source, *target_c, max_distance, verbose, init, estimation,
                           criteria);
}
