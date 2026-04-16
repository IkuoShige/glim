#pragma once

#include <algorithm>
#include <memory>
#include <vector>

#include <gtsam_points/factors/integrated_matching_cost_factor.hpp>
#include <gtsam_points/types/point_cloud.hpp>

namespace gtsam_points {
struct NearestNeighborSearch;
}  // namespace gtsam_points

namespace glim {

/**
 * @brief Exact point cloud downsampling GICP factor.
 *
 * This factor evaluates point-to-distribution GICP with exact KdTree
 * correspondences and optionally replaces the full residual set with a
 * weighted Caratheodory coreset that preserves H, b, and c at the sampling
 * linearization point.
 */
class ExactGICPFactor : public gtsam_points::IntegratedMatchingCostFactor {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<ExactGICPFactor>;

  ExactGICPFactor(
    gtsam::Key target_key,
    gtsam::Key source_key,
    const gtsam_points::PointCloud::ConstPtr& target,
    const gtsam_points::PointCloud::ConstPtr& source,
    const std::shared_ptr<const gtsam_points::NearestNeighborSearch>& target_tree = nullptr);

  ExactGICPFactor(
    const gtsam::Pose3& fixed_target_pose,
    gtsam::Key source_key,
    const gtsam_points::PointCloud::ConstPtr& target,
    const gtsam_points::PointCloud::ConstPtr& source,
    const std::shared_ptr<const gtsam_points::NearestNeighborSearch>& target_tree = nullptr);

  ExactGICPFactor(const ExactGICPFactor& other);
  virtual ~ExactGICPFactor() override;

  virtual void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;
  virtual size_t memory_usage() const override;
  virtual gtsam::NonlinearFactor::shared_ptr clone() const override { return gtsam::NonlinearFactor::shared_ptr(new ExactGICPFactor(*this)); }
  virtual std::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values& values) const override;

  void set_num_threads(int n) { num_threads = std::max(1, n); }
  void set_max_correspondence_distance(double dist) { max_correspondence_distance_sq = dist * dist; }
  void set_enable_exact_downsampling(bool enable) { enable_exact_downsampling = enable; }
  void set_coreset_target_num_points(int n) { coreset_target_num_points = n; }
  void set_coreset_num_clusters(int n) { coreset_num_clusters = std::max(1, n); }
  void set_deferred_sampling_threshold(double translation, double rotation) {
    deferred_sampling_translation = translation;
    deferred_sampling_rotation = rotation;
  }
  void set_rebuild_threshold(double translation, double rotation) {
    rebuild_translation = translation;
    rebuild_rotation = rotation;
  }

  double inlier_fraction() const;

private:
  virtual void update_correspondences(const Eigen::Isometry3d& delta) const override;

  virtual double evaluate(
    const Eigen::Isometry3d& delta,
    Eigen::Matrix<double, 6, 6>* H_target = nullptr,
    Eigen::Matrix<double, 6, 6>* H_source = nullptr,
    Eigen::Matrix<double, 6, 6>* H_target_source = nullptr,
    Eigen::Matrix<double, 6, 1>* b_target = nullptr,
    Eigen::Matrix<double, 6, 1>* b_source = nullptr) const override;

private:
  struct LinearizationResult;
  struct CoresetCache;

  struct WeightedContribution;

  LinearizationResult evaluate_full(const Eigen::Isometry3d& delta, bool build_features, bool need_target = true) const;
  LinearizationResult evaluate_subset(const Eigen::Isometry3d& delta, const std::vector<int>& source_indices, const std::vector<double>& weights, bool need_target = true) const;
  LinearizationResult evaluate_for_linearize(const Eigen::Isometry3d& delta) const;
  void compute_perpoint(int source_index, const Eigen::Isometry3d& delta, const Eigen::Matrix3d& R, WeightedContribution& c, unsigned char& valid_flag, bool need_target = true) const;

  void ensure_target_tree() const;
  bool extract_coreset() const;
  int feature_dim() const;
  bool within_threshold(const Eigen::Isometry3d& delta, const Eigen::Isometry3d& reference, double trans_thresh, double rot_thresh) const;

private:
  int num_threads;
  double max_correspondence_distance_sq;

  bool enable_exact_downsampling;
  int coreset_target_num_points;
  int coreset_num_clusters;
  double deferred_sampling_translation;
  double deferred_sampling_rotation;
  double rebuild_translation;
  double rebuild_rotation;

  gtsam_points::PointCloud::ConstPtr target;
  gtsam_points::PointCloud::ConstPtr source;
  mutable std::shared_ptr<const gtsam_points::NearestNeighborSearch> target_tree;

  mutable std::unique_ptr<CoresetCache> coreset_cache;
  mutable int last_num_inliers;
  mutable int last_num_evaluated;
};

}  // namespace glim
