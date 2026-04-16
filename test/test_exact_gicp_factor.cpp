#include <glim/factors/exact_gicp_factor.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/types/point_cloud_cpu.hpp>

namespace {

gtsam::Pose3 to_pose3(const Eigen::Isometry3d& transform) {
  return gtsam::Pose3(gtsam::Rot3(transform.linear()), transform.translation());
}

[[noreturn]] void fail(const std::string& message) {
  std::cerr << "[test_exact_gicp_factor] " << message << std::endl;
  std::exit(1);
}

gtsam_points::PointCloudCPU::Ptr make_cloud(
  const std::vector<Eigen::Vector4d>& points,
  const std::vector<Eigen::Matrix4d>& covariances) {
  if (points.size() != covariances.size()) {
    fail("point/covariance size mismatch");
  }

  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>();
  cloud->num_points = points.size();
  cloud->points_storage = points;
  cloud->covs_storage = covariances;
  cloud->points = cloud->points_storage.data();
  cloud->covs = cloud->covs_storage.data();
  return cloud;
}

std::vector<Eigen::Matrix4d> make_covariances(size_t num_points, double base) {
  std::vector<Eigen::Matrix4d> covariances(num_points, Eigen::Matrix4d::Zero());
  for (size_t i = 0; i < num_points; i++) {
    const double s = static_cast<double>(i % 7);
    covariances[i](0, 0) = base + 0.001 * s;
    covariances[i](1, 1) = base + 0.003 + 0.0015 * s;
    covariances[i](2, 2) = base + 0.006 + 0.0007 * s;
  }
  return covariances;
}

std::vector<Eigen::Vector4d> make_source_points() {
  std::vector<Eigen::Vector4d> points;
  points.reserve(144);

  for (int i = 0; i < 144; i++) {
    const double gx = static_cast<double>(i % 12) - 5.5;
    const double gy = static_cast<double>((i / 12) % 6) - 2.5;
    const double gz = static_cast<double>(i / 72) - 0.5;

    const double x = 0.37 * gx + 0.015 * std::sin(0.7 * i);
    const double y = 0.33 * gy + 0.011 * std::cos(0.5 * i);
    const double z = 0.29 * gz + 0.027 * std::sin(0.23 * i);
    points.emplace_back(x, y, z, 1.0);
  }

  return points;
}

Eigen::Isometry3d make_source_pose() {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() =
    (Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitY()) *
     Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
  transform.translation() = Eigen::Vector3d(0.42, -0.18, 0.11);
  return transform;
}

std::vector<Eigen::Vector4d> make_target_points(const std::vector<Eigen::Vector4d>& source_points, const Eigen::Isometry3d& source_pose) {
  std::vector<Eigen::Vector4d> points;
  points.reserve(source_points.size());

  for (size_t i = 0; i < source_points.size(); i++) {
    Eigen::Vector4d point = source_pose * source_points[i];
    point[0] += 0.004 * std::sin(0.37 * static_cast<double>(i));
    point[1] += 0.003 * std::cos(0.19 * static_cast<double>(i));
    point[2] += 0.002 * std::sin(0.53 * static_cast<double>(i));
    point[3] = 1.0;
    points.push_back(point);
  }

  return points;
}

std::shared_ptr<gtsam::HessianFactor> linearize_to_hessian(const glim::ExactGICPFactor& factor, const gtsam::Values& values) {
  auto gaussian = factor.linearize(values);
  auto hessian = std::dynamic_pointer_cast<gtsam::HessianFactor>(gaussian);
  if (!hessian) {
    fail("linearize() did not return HessianFactor");
  }
  return hessian;
}

void assert_near_matrix(const Eigen::MatrixXd& expected, const Eigen::MatrixXd& actual, const std::string& label) {
  if (expected.rows() != actual.rows() || expected.cols() != actual.cols()) {
    fail(label + " matrix size mismatch");
  }

  const Eigen::MatrixXd diff = expected - actual;
  const double max_abs = diff.cwiseAbs().maxCoeff();
  const double relative = diff.norm() / std::max(1.0, expected.norm());
  if (max_abs > 1e-6 && relative > 1e-9) {
    std::cerr << "[test_exact_gicp_factor] " << label << " max_abs=" << max_abs << " relative=" << relative << std::endl;
    fail(label + " coreset linearization does not match full linearization");
  }
}

void configure_full(glim::ExactGICPFactor& factor) {
  factor.set_num_threads(2);
  factor.set_max_correspondence_distance(0.25);
  factor.set_enable_exact_downsampling(false);
}

void configure_coreset(glim::ExactGICPFactor& factor) {
  factor.set_num_threads(2);
  factor.set_max_correspondence_distance(0.25);
  factor.set_enable_exact_downsampling(true);
  factor.set_coreset_target_num_points(29);
  factor.set_coreset_num_clusters(32);
  factor.set_deferred_sampling_threshold(1.0, 1.0);
  factor.set_rebuild_threshold(1.0, 1.0);
}

void test_unary_factor(
  const gtsam_points::PointCloud::ConstPtr& target,
  const gtsam_points::PointCloud::ConstPtr& source,
  const Eigen::Isometry3d& source_pose) {
  const gtsam::Key source_key = gtsam::Symbol('x', 0);

  gtsam::Values values;
  values.insert(source_key, to_pose3(source_pose));

  glim::ExactGICPFactor full(to_pose3(Eigen::Isometry3d::Identity()), source_key, target, source);
  configure_full(full);

  glim::ExactGICPFactor coreset(to_pose3(Eigen::Isometry3d::Identity()), source_key, target, source);
  configure_coreset(coreset);

  const Eigen::MatrixXd expected = linearize_to_hessian(full, values)->augmentedInformation();
  linearize_to_hessian(coreset, values);
  const Eigen::MatrixXd actual = linearize_to_hessian(coreset, values)->augmentedInformation();

  assert_near_matrix(expected, actual, "unary factor");
}

void test_binary_factor(
  const gtsam_points::PointCloud::ConstPtr& target,
  const gtsam_points::PointCloud::ConstPtr& source,
  const Eigen::Isometry3d& source_pose) {
  const gtsam::Key target_key = gtsam::Symbol('x', 0);
  const gtsam::Key source_key = gtsam::Symbol('x', 1);

  gtsam::Values values;
  values.insert(target_key, to_pose3(Eigen::Isometry3d::Identity()));
  values.insert(source_key, to_pose3(source_pose));

  glim::ExactGICPFactor full(target_key, source_key, target, source);
  configure_full(full);

  glim::ExactGICPFactor coreset(target_key, source_key, target, source);
  configure_coreset(coreset);

  const Eigen::MatrixXd expected = linearize_to_hessian(full, values)->augmentedInformation();
  linearize_to_hessian(coreset, values);
  const Eigen::MatrixXd actual = linearize_to_hessian(coreset, values)->augmentedInformation();

  assert_near_matrix(expected, actual, "binary factor");
}

}  // namespace

int main() {
  const auto source_points = make_source_points();
  const auto source_covariances = make_covariances(source_points.size(), 0.02);
  const auto source_pose = make_source_pose();
  const auto target_points = make_target_points(source_points, source_pose);
  const auto target_covariances = make_covariances(target_points.size(), 0.025);

  const auto source = make_cloud(source_points, source_covariances);
  const auto target = make_cloud(target_points, target_covariances);

  test_unary_factor(target, source, source_pose);
  test_binary_factor(target, source, source_pose);

  std::cout << "[test_exact_gicp_factor] passed" << std::endl;
  return 0;
}
