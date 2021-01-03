#pragma once

#include <Eigen/Core>
#include <memory>
#include <tuple>
#include <vector>

#include "kdtree_search_param.h"


/// \class PointCloud
///
/// \brief A point cloud consists of point coordinates, and optionally point
/// colors and point normals.
class PointCloud {
public:
    /// \brief Default Constructor.
    PointCloud() {}
    /// \brief Parameterized Constructor.
    ///
    /// \param points Points coordinates.
    PointCloud(const std::string filename);

    PointCloud(const std::vector<Eigen::Vector3d> &points,
               const std::vector<Eigen::Vector3d> &normals = std::vector<Eigen::Vector3d>(),
               const std::vector<Eigen::Vector3d> &colors = std::vector<Eigen::Vector3d>()
             ) : points_(points), normals_(normals) , colors_(colors) {}

    PointCloud(std::vector<Eigen::Vector3d>&& points,
               std::vector<Eigen::Vector3d>&& normals = std::vector<Eigen::Vector3d>(),
               std::vector<Eigen::Vector3d>&& colors = std::vector<Eigen::Vector3d>()
             ) : points_(std::move(points)), normals_(std::move(normals)), colors_(std::move(colors)) {}
    // PointCloud(const PointCloud& other) : points_(other.points_) {}
    ~PointCloud()  {}

public:
    PointCloud &Clear();
    bool IsEmpty() const ;
    Eigen::Vector3d GetMinBound() const ;
    Eigen::Vector3d GetMaxBound() const;
    Eigen::Vector3d GetCenter() const ;
    PointCloud &Transform(const Eigen::Matrix4d &transformation);
    PointCloud &Translate(const Eigen::Vector3d &translation,
                          bool relative = true);
    PointCloud &Scale(const double scale,
                      const Eigen::Vector3d &center) ;
    PointCloud &Rotate(const Eigen::Matrix3d &R,
                       const Eigen::Vector3d &center) ;

    PointCloud &operator+=(const PointCloud &cloud);
    PointCloud operator+(const PointCloud &cloud) const;

    /// Returns 'true' if the point cloud contains points.
    bool HasPoints() const { return points_.size() > 0; }

    /// Returns `true` if the point cloud contains point normals.
    bool HasNormals() const {
        return points_.size() > 0 && normals_.size() == points_.size();
    }

    /// Returns `true` if the point cloud contains point colors.
    bool HasColors() const {
        return points_.size() > 0 && colors_.size() == points_.size();
    }

    /// Normalize point normals to length 1.
    PointCloud &NormalizeNormals() {
        for (size_t i = 0; i < normals_.size(); i++) {
            normals_[i].normalize();
        }
        return *this;
    }

    /// \brief Remove all points from the point cloud that have a nan entry, or
    /// infinite entries.
    ///
    /// Also removes the corresponding normals and color entries.
    ///
    /// \param remove_nan Remove NaN values from the PointCloud.
    /// \param remove_infinite Remove infinite values from the PointCloud.
    PointCloud &RemoveNonFinitePoints(bool remove_nan = true,
                                      bool remove_infinite = true);

    /// \brief Function to select points from \p input pointcloud into
    /// \p output pointcloud.
    ///
    /// Points with indices in \p indices are selected.
    ///
    /// \param indices Indices of points to be selected.
    /// \param invert Set to `True` to invert the selection of indices.
    std::shared_ptr<PointCloud> SelectByIndex(
            const std::vector<size_t> &indices, bool invert = false) const;

    // /// \brief Function to downsample input pointcloud into output pointcloud
    // /// with a voxel.
    // ///
    // /// Normals and colors are averaged if they exist.
    // ///
    // /// \param voxel_size Defines the resolution of the voxel grid,
    // /// smaller value leads to denser output point cloud.
    std::shared_ptr<PointCloud> VoxelDownSample(double voxel_size) const;

    /// \brief Function to compute the point to point distances between point
    /// clouds.
    ///
    /// For each point in the \p source point cloud, compute the distance to the
    /// \p target point cloud.
    ///
    /// \param target The target point cloud.
    std::vector<double> ComputePointCloudDistance(const PointCloud &target);

        // /// \brief Function to compute the normals of a point cloud.
    // ///
    // /// Normals are oriented with respect to the input point cloud if normals
    // /// exist.
    // ///
    // /// \param search_param The KDTree search parameters for neighborhood
    // /// search. \param fast_normal_computation If true, the normal estiamtion
    // /// uses a non-iterative method to extract the eigenvector from the
    // /// covariance matrix. This is faster, but is not as numerical stable.

    void EstimateNormals(
            const KDTreeSearchParam &search_param = KDTreeSearchParamKNN(),
            bool fast_normal_computation = true);


    // /// \brief Function to downsample using geometry.PointCloud.VoxelDownSample
    // ///
    // /// Also records point cloud index before downsampling.
    // ///
    // /// \param voxel_size Voxel size to downsample into.
    // /// \param min_bound Minimum coordinate of voxel boundaries
    // /// \param max_bound Maximum coordinate of voxel boundaries
    // /// \param approximate_class Whether to approximate.
    // std::tuple<std::shared_ptr<PointCloud>,
    //            Eigen::MatrixXi,
    //            std::vector<std::vector<int>>>
    // VoxelDownSampleAndTrace(double voxel_size,
    //                         const Eigen::Vector3d &min_bound,
    //                         const Eigen::Vector3d &max_bound,
    //                         bool approximate_class = false) const;
    //
    // /// \brief Function to downsample input pointcloud into output pointcloud
    // /// uniformly.
    // ///
    // /// The sample is performed in the order of the points with the 0-th point
    // /// always chosen, not at random.
    // ///
    // /// \param every_k_points Sample rate, the selected point indices are [0, k,
    // /// 2k, …].
    // std::shared_ptr<PointCloud> UniformDownSample(size_t every_k_points) const;
    //
    // /// \brief Function to remove points that have less than \p nb_points in a
    // /// sphere of a given radius.
    // ///
    // /// \param nb_points Number of points within the radius.
    // /// \param search_radius Radius of the sphere.
    // std::tuple<std::shared_ptr<PointCloud>, std::vector<size_t>>
    // RemoveRadiusOutliers(size_t nb_points, double search_radius) const;
    //
    // / \brief Function to remove points that are further away from their
    // / \p nb_neighbor neighbors in average.
    // /
    // / \param nb_neighbors Number of neighbors around the target point.
    // / \param std_ratio Standard deviation ratio.
    std::tuple<std::shared_ptr<PointCloud>, std::vector<size_t>>
    RemoveStatisticalOutliers(size_t nb_neighbors, double std_ratio) const;


    //
    // /// Function to compute the mean and covariance matrix
    // /// of a point cloud.
    // std::tuple<Eigen::Vector3d, Eigen::Matrix3d> ComputeMeanAndCovariance() const;
    //
    // /// \brief Function to compute the Mahalanobis distance for points
    // /// in an input point cloud.
    // ///
    // /// See: https://en.wikipedia.org/wiki/Mahalanobis_distance
    // std::vector<double> ComputeMahalanobisDistance() const;
    //
    // /// Function to compute the distance from a point to its nearest neighbor in
    // /// the input point cloud
    // std::vector<double> ComputeNearestNeighborDistance() const;
    //
    // /// Function that computes the convex hull of the point cloud using qhull
    // std::tuple<std::shared_ptr<TriangleMesh>, std::vector<size_t>> ComputeConvexHull() const;
    //
    // /// \brief This is an implementation of the Hidden Point Removal operator
    // /// described in Katz et. al. 'Direct Visibility of Point Sets', 2007.
    // ///
    // /// Additional information about the choice of radius
    // /// for noisy point clouds can be found in Mehra et. al. 'Visibility of
    // /// Noisy Point Cloud Data', 2010.
    // ///
    // /// \param camera_location All points not visible from that location will be
    // /// removed. \param radius The radius of the spherical projection.
    // std::tuple<std::shared_ptr<TriangleMesh>, std::vector<size_t>>
    // HiddenPointRemoval(const Eigen::Vector3d &camera_location,
    //                    const double radius) const;
    //
    // /// \brief Cluster PointCloud using the DBSCAN algorithm
    // /// Ester et al., "A Density-Based Algorithm for Discovering Clusters
    // /// in Large Spatial Databases with Noise", 1996
    // ///
    // /// Returns a list of point labels, -1 indicates noise according to
    // /// the algorithm.
    // ///
    // /// \param eps Density parameter that is used to find neighbouring points.
    // /// \param min_points Minimum number of points to form a cluster.
    // /// \param print_progress If `true` the progress is visualized in the
    // /// console.
    // std::vector<int> ClusterDBSCAN(double eps,
    //                                size_t min_points,
    //                                bool print_progress = false) const;
    //
    // /// \brief Segment PointCloud plane using the RANSAC algorithm.
    // ///
    // /// \param distance_threshold Max distance a point can be from the plane
    // /// model, and still be considered an inlier.
    // /// \param ransac_n Number of initial points to be considered inliers in
    // /// each iteration.
    // /// \param num_iterations Number of iterations.
    // /// \return Returns the plane model ax + by + cz + d = 0 and the indices of
    // /// the plane inliers.
    // std::tuple<Eigen::Vector4d, std::vector<size_t>> SegmentPlane(
    //         const double distance_threshold = 0.01,
    //         const int ransac_n = 3,
    //         const int num_iterations = 100) const;

public:
    /// Points coordinates.
    std::vector<Eigen::Vector3d> points_;
    /// Points normals.
    std::vector<Eigen::Vector3d> normals_;
    /// RGB colors of points.
    std::vector<Eigen::Vector3d> colors_;
};
