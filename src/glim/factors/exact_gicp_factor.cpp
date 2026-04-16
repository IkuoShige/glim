#include <glim/factors/exact_gicp_factor.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>

#include <Eigen/LU>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam_points/ann/kdtree2.hpp>
#include <gtsam_points/ann/nearest_neighbor_search.hpp>
#include <gtsam_points/types/frame_traits.hpp>

namespace glim {

using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Vector6 = Eigen::Matrix<double, 6, 1>;

struct ExactGICPFactor::WeightedContribution {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int source_index = -1;
  double error = 0.0;
  Matrix6 H_target = Matrix6::Zero();
  Matrix6 H_source = Matrix6::Zero();
  Matrix6 H_target_source = Matrix6::Zero();
  Vector6 b_target = Vector6::Zero();
  Vector6 b_source = Vector6::Zero();
};

namespace {

template <typename Derived>
bool all_finite(const Eigen::MatrixBase<Derived>& x) {
  return x.array().isFinite().all();
}

void reduce_caratheodory(
  const Eigen::MatrixXd& P_in,
  const Eigen::VectorXd& u_in,
  Eigen::MatrixXd& S_out,
  Eigen::VectorXd& w_out,
  Eigen::VectorXi& selected,
  int target_num) {
  if (target_num <= 0) {
    target_num = P_in.rows() + 1;
  }

  if (P_in.cols() <= target_num) {
    S_out = P_in;
    w_out = u_in;
    selected.resize(P_in.cols());
    std::iota(selected.data(), selected.data() + selected.size(), 0);
    return;
  }

  Eigen::MatrixXd P = P_in;
  Eigen::VectorXd u = u_in;
  selected.resize(P.cols());
  std::iota(selected.data(), selected.data() + selected.size(), 0);

  int last_cols = -1;
  while (P.cols() > target_num) {
    if (last_cols == P.cols()) {
      break;
    }
    last_cols = P.cols();

    Eigen::MatrixXd A = P.rightCols(P.cols() - 1).colwise() - P.col(0);
    Eigen::FullPivLU<Eigen::MatrixXd> lu(A);
    Eigen::MatrixXd kernel = lu.kernel();
    if (kernel.cols() == 0) {
      break;
    }

    Eigen::VectorXd v(P.cols());
    v.tail(P.cols() - 1) = kernel.col(0);
    v[0] = -v.tail(P.cols() - 1).sum();

    double alpha = std::numeric_limits<double>::max();
    for (int i = 0; i < P.cols(); i++) {
      if (v[i] > 0.0) {
        alpha = std::min(alpha, u[i] / v[i]);
      }
    }

    if (!std::isfinite(alpha) || alpha == std::numeric_limits<double>::max()) {
      break;
    }

    Eigen::VectorXd candidate = u - alpha * v;
    if ((candidate.array() > 0.0).count() == candidate.size()) {
      Eigen::Index min_index = 0;
      candidate.minCoeff(&min_index);
      candidate[min_index] = 0.0;
    }

    const int positives = (candidate.array() > 0.0).count();
    if (positives == 0 || positives >= P.cols()) {
      break;
    }

    Eigen::MatrixXd P_next(P.rows(), positives);
    Eigen::VectorXd u_next(positives);
    Eigen::VectorXi selected_next(positives);

    int cursor = 0;
    for (int i = 0; i < candidate.size(); i++) {
      if (candidate[i] <= 0.0) {
        continue;
      }
      P_next.col(cursor) = P.col(i);
      u_next[cursor] = candidate[i];
      selected_next[cursor] = selected[i];
      cursor++;
    }

    P = std::move(P_next);
    u = std::move(u_next);
    selected = std::move(selected_next);
  }

  S_out = std::move(P);
  w_out = std::move(u);
}

void fast_caratheodory(
  const Eigen::MatrixXd& P_in,
  int num_clusters,
  Eigen::VectorXi& selected,
  Eigen::VectorXd& weights,
  int target_num) {
  if (target_num <= 0) {
    target_num = P_in.rows() + 1;
  }

  if (P_in.cols() <= target_num) {
    selected.resize(P_in.cols());
    weights.setOnes(P_in.cols());
    std::iota(selected.data(), selected.data() + selected.size(), 0);
    return;
  }

  Eigen::MatrixXd P = P_in;
  Eigen::VectorXd u = Eigen::VectorXd::Constant(P.cols(), 1.0 / P.cols());
  Eigen::VectorXi original_indices(P.cols());
  std::iota(original_indices.data(), original_indices.data() + original_indices.size(), 0);

  int last_cols = -1;
  while (P.cols() > target_num) {
    if (last_cols == P.cols()) {
      break;
    }
    last_cols = P.cols();

    const int k = std::max(1, std::min(num_clusters, static_cast<int>(P.cols())));
    std::vector<int> cluster_bounds(k + 1, 0);
    for (int i = 0; i <= k; i++) {
      cluster_bounds[i] = (static_cast<int>(P.cols()) * i + k - 1) / k;
    }

    Eigen::MatrixXd centroids = Eigen::MatrixXd::Zero(P.rows(), k);
    Eigen::VectorXd cluster_weights = Eigen::VectorXd::Zero(k);
    for (int cluster = 0; cluster < k; cluster++) {
      for (int j = cluster_bounds[cluster]; j < cluster_bounds[cluster + 1]; j++) {
        cluster_weights[cluster] += u[j];
        centroids.col(cluster) += u[j] * P.col(j);
      }
      if (cluster_weights[cluster] > 0.0) {
        centroids.col(cluster) /= cluster_weights[cluster];
      }
    }

    int points_per_cluster = std::max(1, cluster_bounds.back() - cluster_bounds[k - 1]);
    int reduced_cluster_num = P.rows() + 1;
    if (reduced_cluster_num * points_per_cluster < target_num) {
      reduced_cluster_num = std::max(1, target_num / points_per_cluster);
    }

    Eigen::MatrixXd reduced_centroids;
    Eigen::VectorXd reduced_weights;
    Eigen::VectorXi kept_clusters;
    reduce_caratheodory(centroids, cluster_weights, reduced_centroids, reduced_weights, kept_clusters, reduced_cluster_num);

    int next_cols = 0;
    for (int kept = 0; kept < kept_clusters.size(); kept++) {
      const int cluster = kept_clusters[kept];
      next_cols += cluster_bounds[cluster + 1] - cluster_bounds[cluster];
    }
    if (next_cols <= 0 || next_cols >= P.cols()) {
      break;
    }

    Eigen::MatrixXd P_next(P.rows(), next_cols);
    Eigen::VectorXd u_next(next_cols);
    Eigen::VectorXi original_next(next_cols);

    int cursor = 0;
    for (int kept = 0; kept < kept_clusters.size(); kept++) {
      const int cluster = kept_clusters[kept];
      const double cluster_weight = cluster_weights[cluster];
      if (cluster_weight <= 0.0) {
        continue;
      }

      for (int j = cluster_bounds[cluster]; j < cluster_bounds[cluster + 1]; j++) {
        P_next.col(cursor) = P.col(j);
        u_next[cursor] = reduced_weights[kept] * u[j] / cluster_weight;
        original_next[cursor] = original_indices[j];
        cursor++;
      }
    }

    P = P_next.leftCols(cursor);
    u = u_next.head(cursor);
    original_indices = original_next.head(cursor);
  }

  selected = original_indices;
  weights = u * P_in.cols();
}

}  // namespace

struct ExactGICPFactor::LinearizationResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Matrix6 H_target = Matrix6::Zero();
  Matrix6 H_source = Matrix6::Zero();
  Matrix6 H_target_source = Matrix6::Zero();
  Vector6 b_target = Vector6::Zero();
  Vector6 b_source = Vector6::Zero();
  double error = 0.0;
  int num_inliers = 0;
  int num_evaluated = 0;
  Eigen::MatrixXd features;
  std::vector<int> feature_source_indices;
};

struct ExactGICPFactor::CoresetCache {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool has_full_sample = false;
  bool has_coreset = false;
  Eigen::Isometry3d full_linearization_point = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d coreset_linearization_point = Eigen::Isometry3d::Identity();
  Eigen::MatrixXd features;
  std::vector<int> feature_source_indices;
  std::vector<int> selected_source_indices;
  std::vector<double> selected_weights;
};

ExactGICPFactor::ExactGICPFactor(
  gtsam::Key target_key,
  gtsam::Key source_key,
  const gtsam_points::PointCloud::ConstPtr& target,
  const gtsam_points::PointCloud::ConstPtr& source,
  const std::shared_ptr<const gtsam_points::NearestNeighborSearch>& target_tree)
: gtsam_points::IntegratedMatchingCostFactor(target_key, source_key),
  num_threads(1),
  max_correspondence_distance_sq(1.0),
  enable_exact_downsampling(true),
  coreset_target_num_points(96),
  coreset_num_clusters(64),
  deferred_sampling_translation(0.25),
  deferred_sampling_rotation(0.25 * M_PI / 180.0),
  rebuild_translation(1.0),
  rebuild_rotation(1.0 * M_PI / 180.0),
  target(target),
  source(source),
  target_tree(target_tree),
  coreset_cache(new CoresetCache),
  last_num_inliers(0),
  last_num_evaluated(0) {
  if (!this->target || !this->source || !this->target->has_points() || !this->source->has_points() || !this->target->has_covs() || !this->source->has_covs()) {
    std::cerr << "error: ExactGICPFactor requires target/source points and covariances" << std::endl;
    abort();
  }

}

ExactGICPFactor::ExactGICPFactor(
  const gtsam::Pose3& fixed_target_pose,
  gtsam::Key source_key,
  const gtsam_points::PointCloud::ConstPtr& target,
  const gtsam_points::PointCloud::ConstPtr& source,
  const std::shared_ptr<const gtsam_points::NearestNeighborSearch>& target_tree)
: gtsam_points::IntegratedMatchingCostFactor(fixed_target_pose, source_key),
  num_threads(1),
  max_correspondence_distance_sq(1.0),
  enable_exact_downsampling(true),
  coreset_target_num_points(96),
  coreset_num_clusters(64),
  deferred_sampling_translation(0.25),
  deferred_sampling_rotation(0.25 * M_PI / 180.0),
  rebuild_translation(1.0),
  rebuild_rotation(1.0 * M_PI / 180.0),
  target(target),
  source(source),
  target_tree(target_tree),
  coreset_cache(new CoresetCache),
  last_num_inliers(0),
  last_num_evaluated(0) {
  if (!this->target || !this->source || !this->target->has_points() || !this->source->has_points() || !this->target->has_covs() || !this->source->has_covs()) {
    std::cerr << "error: ExactGICPFactor requires target/source points and covariances" << std::endl;
    abort();
  }

}

ExactGICPFactor::ExactGICPFactor(const ExactGICPFactor& other)
: gtsam_points::IntegratedMatchingCostFactor(other),
  num_threads(other.num_threads),
  max_correspondence_distance_sq(other.max_correspondence_distance_sq),
  enable_exact_downsampling(other.enable_exact_downsampling),
  coreset_target_num_points(other.coreset_target_num_points),
  coreset_num_clusters(other.coreset_num_clusters),
  deferred_sampling_translation(other.deferred_sampling_translation),
  deferred_sampling_rotation(other.deferred_sampling_rotation),
  rebuild_translation(other.rebuild_translation),
  rebuild_rotation(other.rebuild_rotation),
  target(other.target),
  source(other.source),
  target_tree(other.target_tree),
  coreset_cache(new CoresetCache(*other.coreset_cache)),
  last_num_inliers(other.last_num_inliers),
  last_num_evaluated(other.last_num_evaluated) {}

ExactGICPFactor::~ExactGICPFactor() {}

void ExactGICPFactor::print(const std::string& s, const gtsam::KeyFormatter& keyFormatter) const {
  std::cout << s << "ExactGICPFactor";
  if (is_binary) {
    std::cout << "(" << keyFormatter(this->keys()[0]) << ", " << keyFormatter(this->keys()[1]) << ")";
  } else {
    std::cout << "(fixed, " << keyFormatter(this->keys()[0]) << ")";
  }
  std::cout << " |target|=" << target->size() << " |source|=" << source->size() << " coreset=" << enable_exact_downsampling
            << " target_num=" << coreset_target_num_points << std::endl;
}

size_t ExactGICPFactor::memory_usage() const {
  size_t bytes = sizeof(*this);
  if (coreset_cache) {
    bytes += sizeof(*coreset_cache);
    bytes += sizeof(double) * coreset_cache->features.size();
    bytes += sizeof(int) * coreset_cache->feature_source_indices.capacity();
    bytes += sizeof(int) * coreset_cache->selected_source_indices.capacity();
    bytes += sizeof(double) * coreset_cache->selected_weights.capacity();
  }
  return bytes;
}

double ExactGICPFactor::inlier_fraction() const {
  return last_num_evaluated > 0 ? static_cast<double>(last_num_inliers) / last_num_evaluated : 0.0;
}

void ExactGICPFactor::update_correspondences(const Eigen::Isometry3d&) const {}

void ExactGICPFactor::ensure_target_tree() const {
  if (!target_tree) {
    target_tree.reset(new gtsam_points::KdTree2<gtsam_points::PointCloud>(target, num_threads));
  }
}

// Shared per-point GICP computation using 3x3 matrices only.
// Avoids unnecessary 4x4 arithmetic since the 4th row/col of points and covs are zero-padded.
void ExactGICPFactor::compute_perpoint(
  int source_index,
  const Eigen::Isometry3d& delta,
  const Eigen::Matrix3d& R,
  WeightedContribution& c,
  unsigned char& valid_flag,
  bool need_target) const {
  const Eigen::Vector3d mean_A = source->points[source_index].head<3>();
  const Eigen::Matrix3d cov_A = source->covs[source_index].topLeftCorner<3, 3>();
  const Eigen::Vector3d transed_mean_A = R * mean_A + delta.translation();

  size_t target_index = 0;
  double sq_dist = 0.0;
  const Eigen::Vector4d transed_4d(transed_mean_A.x(), transed_mean_A.y(), transed_mean_A.z(), 1.0);
  const size_t found = target_tree->knn_search(transed_4d.data(), 1, &target_index, &sq_dist, max_correspondence_distance_sq);
  if (!found || sq_dist >= max_correspondence_distance_sq) {
    return;
  }

  const Eigen::Vector3d mean_B = target->points[target_index].head<3>();
  const Eigen::Matrix3d cov_B = target->covs[target_index].topLeftCorner<3, 3>();
  const Eigen::Vector3d residual3 = mean_B - transed_mean_A;

  const Eigen::Matrix3d RCR = cov_B + R * cov_A * R.transpose();
  bool invertible = false;
  Eigen::Matrix3d maha3;
  RCR.computeInverseWithCheck(maha3, invertible);
  if (!invertible || !all_finite(maha3)) {
    return;
  }

  // Source Jacobian (always needed)
  Eigen::Matrix<double, 3, 6> J_source;
  J_source.leftCols<3>() = R * gtsam::SO3::Hat(mean_A);
  J_source.rightCols<3>() = -R;

  const Eigen::Matrix<double, 6, 3> JsM = J_source.transpose() * maha3;

  c.source_index = source_index;
  c.error = residual3.transpose() * maha3 * residual3;
  c.H_source.noalias() = JsM * J_source;
  c.b_source.noalias() = JsM * residual3;

  // Target Jacobian (only needed for binary factors)
  if (need_target) {
    Eigen::Matrix<double, 3, 6> J_target;
    J_target.leftCols<3>() = -gtsam::SO3::Hat(transed_mean_A);
    J_target.rightCols<3>() = Eigen::Matrix3d::Identity();

    const Eigen::Matrix<double, 6, 3> JtM = J_target.transpose() * maha3;
    c.H_target.noalias() = JtM * J_target;
    c.H_target_source.noalias() = JtM * J_source;
    c.b_target.noalias() = JtM * residual3;
  }

  if (std::isfinite(c.error) && all_finite(c.H_source) && all_finite(c.b_source)) {
    valid_flag = 1;
  }
}

ExactGICPFactor::LinearizationResult ExactGICPFactor::evaluate_subset(
  const Eigen::Isometry3d& delta,
  const std::vector<int>& source_indices,
  const std::vector<double>& weights,
  bool need_target) const {
  ensure_target_tree();

  LinearizationResult result;
  result.num_evaluated = source_indices.size();
  if (source_indices.empty()) {
    return result;
  }

  std::vector<WeightedContribution> contributions(source_indices.size());
  std::vector<unsigned char> valid(source_indices.size(), 0);

  const Eigen::Matrix3d R = delta.linear();

#pragma omp parallel for num_threads(num_threads) schedule(guided, 8)
  for (int i = 0; i < static_cast<int>(source_indices.size()); i++) {
    const int source_index = source_indices[i];
    if (source_index >= 0 && source_index < static_cast<int>(source->size())) {
      compute_perpoint(source_index, delta, R, contributions[i], valid[i], need_target);
    }
  }

  for (int i = 0; i < static_cast<int>(contributions.size()); i++) {
    if (!valid[i]) {
      continue;
    }

    const double weight = weights.empty() ? 1.0 : weights[i];
    if (weight <= 0.0 || !std::isfinite(weight)) {
      continue;
    }

    result.num_inliers++;
    result.error += weight * contributions[i].error;
    result.H_target += weight * contributions[i].H_target;
    result.H_source += weight * contributions[i].H_source;
    result.H_target_source += weight * contributions[i].H_target_source;
    result.b_target += weight * contributions[i].b_target;
    result.b_source += weight * contributions[i].b_source;
  }

  return result;
}

ExactGICPFactor::LinearizationResult ExactGICPFactor::evaluate_full(const Eigen::Isometry3d& delta, bool build_features, bool need_target) const {
  ensure_target_tree();

  const int n = static_cast<int>(source->size());

  LinearizationResult result;
  result.num_evaluated = n;
  if (n == 0) {
    return result;
  }

  std::vector<WeightedContribution> contributions(n);
  std::vector<unsigned char> valid(n, 0);

  const Eigen::Matrix3d R = delta.linear();

#pragma omp parallel for num_threads(num_threads) schedule(guided, 8)
  for (int i = 0; i < n; i++) {
    compute_perpoint(i, delta, R, contributions[i], valid[i], need_target);
  }

  int num_feature_cols = 0;
  if (build_features) {
    for (unsigned char is_valid : valid) {
      num_feature_cols += is_valid ? 1 : 0;
    }
    result.features.resize(feature_dim(), num_feature_cols);
    result.feature_source_indices.reserve(num_feature_cols);
  }

  int feature_cursor = 0;
  for (int i = 0; i < static_cast<int>(contributions.size()); i++) {
    if (!valid[i]) {
      continue;
    }

    result.num_inliers++;
    result.error += contributions[i].error;
    result.H_target += contributions[i].H_target;
    result.H_source += contributions[i].H_source;
    result.H_target_source += contributions[i].H_target_source;
    result.b_target += contributions[i].b_target;
    result.b_source += contributions[i].b_source;

    if (build_features) {
      // Extract 28-dim feature: upper-triangle of H_source (21), b_source (6), error (1)
      {
        Eigen::VectorXd feature(28);
        int fc = 0;
        for (int r = 0; r < 6; r++) {
          for (int col = r; col < 6; col++) {
            feature[fc++] = contributions[i].H_source(r, col);
          }
        }
        feature.middleRows(fc, 6) = contributions[i].b_source;
        fc += 6;
        feature[fc] = contributions[i].error;
        result.features.col(feature_cursor) = feature;
      }
      result.feature_source_indices.push_back(contributions[i].source_index);
      feature_cursor++;
    }
  }

  return result;
}

bool ExactGICPFactor::within_threshold(const Eigen::Isometry3d& delta, const Eigen::Isometry3d& reference, double trans_thresh, double rot_thresh) const {
  const Eigen::Isometry3d diff = reference.inverse() * delta;
  const double diff_trans = diff.translation().norm();
  const double diff_rot = Eigen::AngleAxisd(diff.linear()).angle();
  return diff_trans < trans_thresh && diff_rot < rot_thresh;
}

int ExactGICPFactor::feature_dim() const {
  // Binary GICP factors are functions of the 6-DoF relative pose.
  // With GTSAM's Pose3 tangent convention, J_target = J_source * (-Adj(delta.inverse())),
  // so preserving H_source, b_source, and c exactly also preserves the full binary Hessian at the sampling pose.
  return 28;
}

bool ExactGICPFactor::extract_coreset() const {
  if (!enable_exact_downsampling || !coreset_cache->has_full_sample || coreset_cache->features.cols() == 0) {
    return false;
  }

  const int min_target_num = feature_dim() + 1;
  const int target_num = std::max(coreset_target_num_points, min_target_num);
  if (coreset_cache->features.cols() <= target_num) {
    coreset_cache->selected_source_indices = coreset_cache->feature_source_indices;
    coreset_cache->selected_weights.assign(coreset_cache->selected_source_indices.size(), 1.0);
    coreset_cache->has_coreset = false;
    return false;
  }

  Eigen::VectorXi coreset_indices;
  Eigen::VectorXd weights;
  fast_caratheodory(coreset_cache->features, coreset_num_clusters, coreset_indices, weights, target_num);

  coreset_cache->selected_source_indices.clear();
  coreset_cache->selected_weights.clear();
  coreset_cache->selected_source_indices.reserve(coreset_indices.size());
  coreset_cache->selected_weights.reserve(coreset_indices.size());

  for (int i = 0; i < coreset_indices.size(); i++) {
    const int feature_index = coreset_indices[i];
    if (feature_index < 0 || feature_index >= static_cast<int>(coreset_cache->feature_source_indices.size()) || weights[i] <= 0.0 || !std::isfinite(weights[i])) {
      continue;
    }
    coreset_cache->selected_source_indices.push_back(coreset_cache->feature_source_indices[feature_index]);
    coreset_cache->selected_weights.push_back(weights[i]);
  }

  coreset_cache->coreset_linearization_point = coreset_cache->full_linearization_point;
  coreset_cache->has_coreset = !coreset_cache->selected_source_indices.empty();
  coreset_cache->features.resize(0, 0);
  coreset_cache->feature_source_indices.clear();
  return coreset_cache->has_coreset;
}

ExactGICPFactor::LinearizationResult ExactGICPFactor::evaluate_for_linearize(const Eigen::Isometry3d& delta) const {
  // For unary factors, skip target Hessian computation (~40% per-point savings)
  const bool need_target = is_binary;

  if (!enable_exact_downsampling) {
    return evaluate_full(delta, false, need_target);
  }

  if (coreset_cache->has_coreset &&
      within_threshold(delta, coreset_cache->coreset_linearization_point, rebuild_translation, rebuild_rotation)) {
    return evaluate_subset(delta, coreset_cache->selected_source_indices, coreset_cache->selected_weights, need_target);
  }

  if (coreset_cache->has_full_sample &&
      within_threshold(delta, coreset_cache->full_linearization_point, deferred_sampling_translation, deferred_sampling_rotation)) {
    if (extract_coreset()) {
      return evaluate_subset(delta, coreset_cache->selected_source_indices, coreset_cache->selected_weights, need_target);
    }
  }

  auto result = evaluate_full(delta, true, need_target);
  coreset_cache->has_full_sample = true;
  coreset_cache->has_coreset = false;
  coreset_cache->full_linearization_point = delta;
  coreset_cache->coreset_linearization_point = delta;
  coreset_cache->features = result.features;
  coreset_cache->feature_source_indices = result.feature_source_indices;
  coreset_cache->selected_source_indices.clear();
  coreset_cache->selected_weights.clear();
  return result;
}

double ExactGICPFactor::evaluate(
  const Eigen::Isometry3d& delta,
  Eigen::Matrix<double, 6, 6>* H_target,
  Eigen::Matrix<double, 6, 6>* H_source,
  Eigen::Matrix<double, 6, 6>* H_target_source,
  Eigen::Matrix<double, 6, 1>* b_target,
  Eigen::Matrix<double, 6, 1>* b_source) const {
  auto result = evaluate_full(delta, false);

  last_num_inliers = result.num_inliers;
  last_num_evaluated = result.num_evaluated;

  if (H_target) {
    *H_target = result.H_target;
  }
  if (H_source) {
    *H_source = result.H_source;
  }
  if (H_target_source) {
    *H_target_source = result.H_target_source;
  }
  if (b_target) {
    *b_target = result.b_target;
  }
  if (b_source) {
    *b_source = result.b_source;
  }

  return result.error;
}

std::shared_ptr<gtsam::GaussianFactor> ExactGICPFactor::linearize(const gtsam::Values& values) const {
  const Eigen::Isometry3d delta = calc_delta(values);
  const auto result = evaluate_for_linearize(delta);

  last_num_inliers = result.num_inliers;
  last_num_evaluated = result.num_evaluated;

  if (is_binary) {
    return std::make_shared<gtsam::HessianFactor>(
      keys()[0],
      keys()[1],
      result.H_target,
      result.H_target_source,
      -result.b_target,
      result.H_source,
      -result.b_source,
      result.error);
  }

  return std::make_shared<gtsam::HessianFactor>(keys()[0], result.H_source, -result.b_source, result.error);
}

}  // namespace glim
