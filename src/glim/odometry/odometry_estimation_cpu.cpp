#include <glim/odometry/odometry_estimation_cpu.hpp>

#include <spdlog/spdlog.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <unordered_map>

#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/ann/kdtree2.hpp>
#include <gtsam_points/ann/fast_occupancy_grid.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <glim/factors/exact_gicp_factor.hpp>
#include <glim/util/config.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>

#include <glim/odometry/callbacks.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/task_arena.h>
#endif

namespace glim {

using Callbacks = OdometryEstimationCallbacks;

using gtsam::symbol_shorthand::B;  // IMU bias
using gtsam::symbol_shorthand::V;  // IMU velocity   (v_world_imu)
using gtsam::symbol_shorthand::X;  // IMU pose       (T_world_imu)

OdometryEstimationCPUParams::OdometryEstimationCPUParams() : OdometryEstimationIMUParams() {
  // odometry config
  Config config(GlobalConfig::get_config_path("config_odometry"));

  registration_type = config.param<std::string>("odometry_estimation", "registration_type", "VGICP");
  max_iterations = config.param<int>("odometry_estimation", "max_iterations", 5);
  lru_thresh = config.param<int>("odometry_estimation", "lru_thresh", 100);
  target_downsampling_rate = config.param<double>("odometry_estimation", "target_downsampling_rate", 0.1);

  ivox_resolution = config.param<double>("odometry_estimation", "ivox_resolution", 0.5);
  ivox_min_dist = config.param<double>("odometry_estimation", "ivox_min_dist", 0.1);

  vgicp_resolution = config.param<double>("odometry_estimation", "vgicp_resolution", 0.2);
  vgicp_voxelmap_levels = config.param<int>("odometry_estimation", "vgicp_voxelmap_levels", 2);
  vgicp_voxelmap_scaling_factor = config.param<double>("odometry_estimation", "vgicp_voxelmap_scaling_factor", 2.0);

  epcd_voxel_resolution = config.param<double>("odometry_estimation", "epcd_voxel_resolution", 0.5);
  epcd_voxel_resolution_max = config.param<double>("odometry_estimation", "epcd_voxel_resolution_max", epcd_voxel_resolution);
  epcd_voxel_resolution_dmin = config.param<double>("odometry_estimation", "epcd_voxel_resolution_dmin", 4.0);
  epcd_voxel_resolution_dmax = config.param<double>("odometry_estimation", "epcd_voxel_resolution_dmax", 12.0);
  epcd_voxelmap_levels = config.param<int>("odometry_estimation", "epcd_voxelmap_levels", 2);
  epcd_voxelmap_scaling_factor = config.param<double>("odometry_estimation", "epcd_voxelmap_scaling_factor", 2.0);
  epcd_max_num_keyframes = config.param<int>("odometry_estimation", "epcd_max_num_keyframes", 10);
  epcd_full_connection_window_size = config.param<int>("odometry_estimation", "epcd_full_connection_window_size", 3);

  const std::string epcd_strategy = config.param<std::string>("odometry_estimation", "epcd_keyframe_update_strategy", "OVERLAP");
  if (epcd_strategy == "OVERLAP") {
    epcd_keyframe_strategy = KeyframeUpdateStrategy::OVERLAP;
  } else if (epcd_strategy == "DISPLACEMENT") {
    epcd_keyframe_strategy = KeyframeUpdateStrategy::DISPLACEMENT;
  } else {
    spdlog::warn("unknown EPCD keyframe update strategy {}. Use OVERLAP.", epcd_strategy);
    epcd_keyframe_strategy = KeyframeUpdateStrategy::OVERLAP;
  }

  epcd_keyframe_min_overlap = config.param<double>("odometry_estimation", "epcd_keyframe_min_overlap", 0.1);
  epcd_keyframe_max_overlap = config.param<double>("odometry_estimation", "epcd_keyframe_max_overlap", 0.9);
  epcd_keyframe_delta_trans = config.param<double>("odometry_estimation", "epcd_keyframe_delta_trans", 1.0);
  epcd_keyframe_delta_rot = config.param<double>("odometry_estimation", "epcd_keyframe_delta_rot", 0.25);
  epcd_max_correspondence_distance = config.param<double>("odometry_estimation", "epcd_max_correspondence_distance", 1.0);
  epcd_enable_exact_downsampling = config.param<bool>("odometry_estimation", "epcd_enable_exact_downsampling", true);
  epcd_coreset_target_num_points = config.param<int>("odometry_estimation", "epcd_coreset_target_num_points", 96);
  epcd_coreset_num_clusters = config.param<int>("odometry_estimation", "epcd_coreset_num_clusters", 64);
  epcd_deferred_sampling_translation = config.param<double>("odometry_estimation", "epcd_deferred_sampling_translation", 0.25);
  epcd_deferred_sampling_rotation = config.param<double>("odometry_estimation", "epcd_deferred_sampling_rotation", 0.25 * M_PI / 180.0);
  epcd_rebuild_translation = config.param<double>("odometry_estimation", "epcd_rebuild_translation", 1.0);
  epcd_rebuild_rotation = config.param<double>("odometry_estimation", "epcd_rebuild_rotation", 1.0 * M_PI / 180.0);
}

OdometryEstimationCPUParams::~OdometryEstimationCPUParams() {}

OdometryEstimationCPU::OdometryEstimationCPU(const OdometryEstimationCPUParams& params) : OdometryEstimationIMU(std::make_unique<OdometryEstimationCPUParams>(params)) {
  last_T_target_imu.setIdentity();
  if (params.registration_type == "GICP") {
    target_ivox.reset(new gtsam_points::iVox(params.ivox_resolution));
    target_ivox->voxel_insertion_setting().set_min_dist_in_cell(params.ivox_min_dist);
    target_ivox->set_lru_horizon(params.lru_thresh);
    target_ivox->set_neighbor_voxel_mode(1);
  } else if (params.registration_type == "VGICP") {
    target_voxelmaps.resize(params.vgicp_voxelmap_levels);
    for (int i = 0; i < params.vgicp_voxelmap_levels; i++) {
      const double resolution = params.vgicp_resolution * std::pow(params.vgicp_voxelmap_scaling_factor, i);
      target_voxelmaps[i] = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
      target_voxelmaps[i]->set_lru_horizon(params.lru_thresh);
    }
  } else if (params.registration_type == "EPCD_GICP") {
    logger->info("odometry_estimation_cpu uses EPCD_GICP");
  } else {
    spdlog::error("unknown registration type for odometry_estimation_cpu ({})", params.registration_type);
    abort();
  }
}

OdometryEstimationCPU::~OdometryEstimationCPU() {}

void OdometryEstimationCPU::create_frame(EstimationFrame::Ptr& new_frame) {
  const auto params = static_cast<const OdometryEstimationCPUParams*>(this->params.get());
  if (params->registration_type != "EPCD_GICP" || !new_frame->frame || !new_frame->frame->size()) {
    return;
  }

  const int max_scan_count = 256;
  const double dist_median = gtsam_points::median_distance(new_frame->frame, max_scan_count);
  const double denom = std::max(1e-6, params->epcd_voxel_resolution_dmax - params->epcd_voxel_resolution_dmin);
  const double p = std::max(0.0, std::min(1.0, (dist_median - params->epcd_voxel_resolution_dmin) / denom));
  const double base_resolution = params->epcd_voxel_resolution + p * (params->epcd_voxel_resolution_max - params->epcd_voxel_resolution);

  new_frame->voxelmaps.clear();
  for (int i = 0; i < params->epcd_voxelmap_levels; i++) {
    const double resolution = base_resolution * std::pow(params->epcd_voxelmap_scaling_factor, i);
    auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
    voxelmap->insert(*new_frame->frame);
    new_frame->voxelmaps.push_back(voxelmap);
  }

  // Build FastOccupancyGrid for bit-packed overlap estimation (paper Sec. III-B)
  const double occ_resolution = base_resolution * std::pow(params->epcd_voxelmap_scaling_factor, params->epcd_voxelmap_levels - 1);
  auto occ_grid = std::make_shared<gtsam_points::FastOccupancyGrid>(occ_resolution);
  occ_grid->insert(*new_frame->frame);
  new_frame->custom_data["occ_grid"] = occ_grid;
}

gtsam::NonlinearFactorGraph OdometryEstimationCPU::create_factors(const int current, const gtsam_points::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) {
  const auto params = static_cast<const OdometryEstimationCPUParams*>(this->params.get());
  const int last = current - 1;

  if (params->registration_type == "EPCD_GICP") {
    if (current == 0 || !frames[current]->frame || !frames[current]->frame->size()) {
      return gtsam::NonlinearFactorGraph();
    }

    const auto configure_factor = [params](const std::shared_ptr<ExactGICPFactor>& factor) {
      factor->set_num_threads(params->num_threads);
      factor->set_max_correspondence_distance(params->epcd_max_correspondence_distance);
      factor->set_enable_exact_downsampling(params->epcd_enable_exact_downsampling);
      factor->set_coreset_target_num_points(params->epcd_coreset_target_num_points);
      factor->set_coreset_num_clusters(params->epcd_coreset_num_clusters);
      factor->set_deferred_sampling_threshold(params->epcd_deferred_sampling_translation, params->epcd_deferred_sampling_rotation);
      factor->set_rebuild_threshold(params->epcd_rebuild_translation, params->epcd_rebuild_rotation);
    };

    // Cache KdTrees per target point cloud to avoid redundant O(n log n) builds
    std::unordered_map<const gtsam_points::PointCloud*, std::shared_ptr<const gtsam_points::NearestNeighborSearch>> kdtree_cache;
    const auto get_or_build_kdtree = [&](const gtsam_points::PointCloud::ConstPtr& cloud) -> std::shared_ptr<const gtsam_points::NearestNeighborSearch> {
      auto it = kdtree_cache.find(cloud.get());
      if (it != kdtree_cache.end()) {
        return it->second;
      }
      auto tree = std::make_shared<gtsam_points::KdTree2<gtsam_points::PointCloud>>(cloud, params->num_threads);
      kdtree_cache[cloud.get()] = tree;
      return tree;
    };

    const auto create_binary_factor = [&](gtsam::NonlinearFactorGraph& factors, gtsam::Key target_key, gtsam::Key source_key, const EstimationFrame::ConstPtr& target, const EstimationFrame::ConstPtr& source) {
      if (!target || !source || !target->frame || !source->frame || !target->frame->size() || !source->frame->size()) {
        return;
      }

      auto target_tree = get_or_build_kdtree(target->frame);
      auto factor = gtsam::make_shared<ExactGICPFactor>(target_key, source_key, target->frame, source->frame, target_tree);
      configure_factor(factor);
      factors.add(factor);
    };

    const auto create_unary_factor = [&](gtsam::NonlinearFactorGraph& factors, const gtsam::Pose3& fixed_target_pose, gtsam::Key source_key, const EstimationFrame::ConstPtr& target, const EstimationFrame::ConstPtr& source) {
      if (!target || !source || !target->frame || !source->frame || !target->frame->size() || !source->frame->size()) {
        return;
      }

      auto target_tree = get_or_build_kdtree(target->frame);
      auto factor = gtsam::make_shared<ExactGICPFactor>(fixed_target_pose, source_key, target->frame, source->frame, target_tree);
      configure_factor(factor);
      factors.add(factor);
    };

    gtsam::NonlinearFactorGraph factors;

    for (int target = current - params->epcd_full_connection_window_size; target < current; target++) {
      if (target < 0) {
        continue;
      }
      create_binary_factor(factors, X(target), X(current), frames[target], frames[current]);
    }

    for (const auto& keyframe : keyframes) {
      if (keyframe->id >= current - params->epcd_full_connection_window_size) {
        continue;
      }

      const double span = frames[current]->stamp - keyframe->stamp;
      if (span > params->smoother_lag - 0.1 || !frames[keyframe->id]) {
        const gtsam::Pose3 key_T_world_imu(keyframe->T_world_imu.matrix());
        create_unary_factor(factors, key_T_world_imu, X(current), keyframe, frames[current]);
      } else {
        const int target = keyframe->id;
        create_binary_factor(factors, X(target), X(current), frames[target], frames[current]);
      }
    }

    return factors;
  }

  if (current == 0) {
    last_T_target_imu = frames[current]->T_world_imu;
    update_target(current, frames[current]->T_world_imu);
    return gtsam::NonlinearFactorGraph();
  }

  const Eigen::Isometry3d pred_T_last_current = frames[last]->T_world_imu.inverse() * frames[current]->T_world_imu;
  const Eigen::Isometry3d pred_T_target_imu = last_T_target_imu * pred_T_last_current;

  gtsam::Values values;
  values.insert(X(current), gtsam::Pose3(pred_T_target_imu.matrix()));

  // Create frame-to-model matching factor
  gtsam::NonlinearFactorGraph matching_cost_factors;
  if (params->registration_type == "GICP") {
    auto gicp_factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor_<gtsam_points::iVox, gtsam_points::PointCloud>>(
      gtsam::Pose3(),
      X(current),
      target_ivox,
      frames[current]->frame,
      target_ivox);
    gicp_factor->set_max_correspondence_distance(params->ivox_resolution * 2.0);
    gicp_factor->set_num_threads(params->num_threads);
    matching_cost_factors.add(gicp_factor);
  } else if (params->registration_type == "VGICP") {
    for (const auto& voxelmap : target_voxelmaps) {
      auto vgicp_factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), X(current), voxelmap, frames[current]->frame);
      vgicp_factor->set_num_threads(params->num_threads);
      matching_cost_factors.add(vgicp_factor);
    }
  }

  gtsam::NonlinearFactorGraph graph;
  graph.add(matching_cost_factors);

  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(params->max_iterations);
  lm_params.setAbsoluteErrorTol(0.1);

  gtsam::Pose3 last_estimate = values.at<gtsam::Pose3>(X(current));
  lm_params.termination_criteria = [&](const gtsam::Values& values) {
    const gtsam::Pose3 current_pose = values.at<gtsam::Pose3>(X(current));
    const gtsam::Pose3 delta = last_estimate.inverse() * current_pose;

    const double delta_t = delta.translation().norm();
    const double delta_r = Eigen::AngleAxisd(delta.rotation().matrix()).angle();
    last_estimate = current_pose;

    if (delta_t < 1e-10 && delta_r < 1e-10) {
      // Maybe failed to solve the linear system
      return false;
    }

    // Convergence check
    return delta_t < 1e-3 && delta_r < 1e-3 * M_PI / 180.0;
  };

  // Optimize
  // lm_params.setDiagonalDamping(true);
  gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);

#ifdef GTSAM_USE_TBB
  auto arena = static_cast<tbb::task_arena*>(this->tbb_task_arena.get());
  arena->execute([&] {
#endif
    values = optimizer.optimize();
#ifdef GTSAM_USE_TBB
  });
#endif

  const Eigen::Isometry3d T_target_imu = Eigen::Isometry3d(values.at<gtsam::Pose3>(X(current)).matrix());
  Eigen::Isometry3d T_last_current = last_T_target_imu.inverse() * T_target_imu;
  T_last_current.linear() = Eigen::Quaterniond(T_last_current.linear()).normalized().toRotationMatrix();
  frames[current]->T_world_imu = frames[last]->T_world_imu * T_last_current;
  new_values.insert_or_assign(X(current), gtsam::Pose3(frames[current]->T_world_imu.matrix()));

  gtsam::NonlinearFactorGraph factors;

  // Get linearized matching cost factors
  // const auto linearized = optimizer.last_linearized();
  // for (int i = linearized->size() - matching_cost_factors.size(); i < linearized->size(); i++) {
  //   factors.emplace_shared<gtsam::LinearContainerFactor>(linearized->at(i), values);
  // }

  // TODO: Extract a relative pose covariance from a frame-to-model matching result?
  factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), gtsam::Pose3(T_last_current.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e3));
  factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(current), gtsam::Pose3(T_target_imu.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e3));

  update_target(current, T_target_imu);
  last_T_target_imu = T_target_imu;

  return factors;
}

void OdometryEstimationCPU::fallback_smoother() {}

void OdometryEstimationCPU::update_frames(const int current, const gtsam::NonlinearFactorGraph& new_factors) {
  OdometryEstimationIMU::update_frames(current, new_factors);

  const auto params = static_cast<const OdometryEstimationCPUParams*>(this->params.get());
  if (params->registration_type != "EPCD_GICP") {
    return;
  }

  switch (params->epcd_keyframe_strategy) {
    case OdometryEstimationCPUParams::KeyframeUpdateStrategy::OVERLAP:
      update_keyframes_overlap(current);
      break;
    case OdometryEstimationCPUParams::KeyframeUpdateStrategy::DISPLACEMENT:
      update_keyframes_displacement(current);
      break;
  }

  Callbacks::on_update_keyframes(keyframes);
}

void OdometryEstimationCPU::update_keyframes_overlap(int current) {
  const auto params = static_cast<const OdometryEstimationCPUParams*>(this->params.get());

  if (!frames[current]->frame || !frames[current]->frame->size()) {
    return;
  }

  if (keyframes.empty()) {
    keyframes.push_back(frames[current]);
    return;
  }

  // Helper: get or fallback overlap using FastOccupancyGrid (bit-packed, cache-friendly)
  const auto fast_overlap = [](const EstimationFrame::ConstPtr& target, const gtsam_points::PointCloud::ConstPtr& source, const Eigen::Isometry3d& delta) -> double {
    auto occ = target->get_custom_data<gtsam_points::FastOccupancyGrid>("occ_grid");
    if (occ) {
      return occ->calc_overlap_rate(*source, delta);
    }
    // Fallback to voxelmap
    if (!target->voxelmaps.empty()) {
      return gtsam_points::overlap_auto(target->voxelmaps.back(), source, delta);
    }
    return 0.0;
  };

  // Helper: union overlap against multiple targets
  const auto fast_overlap_multi = [](const std::vector<EstimationFrame::ConstPtr>& targets, const gtsam_points::PointCloud::ConstPtr& source, const std::vector<Eigen::Isometry3d>& deltas) -> double {
    if (targets.empty() || !source || !source->size()) {
      return 0.0;
    }
    // Build union occupancy grid from all targets
    const auto* first_occ = targets[0]->get_custom_data<gtsam_points::FastOccupancyGrid>("occ_grid");
    if (!first_occ) {
      // Fallback: use voxelmap path
      std::vector<gtsam_points::GaussianVoxelMap::ConstPtr> vmaps;
      for (const auto& t : targets) {
        if (!t->voxelmaps.empty()) vmaps.push_back(t->voxelmaps.back());
      }
      return vmaps.empty() ? 0.0 : gtsam_points::overlap_auto(vmaps, source, deltas);
    }
    // Use first target's grid resolution for the union grid
    // Build a temporary grid with all target points
    auto union_grid = std::make_shared<gtsam_points::FastOccupancyGrid>(0.5);  // default resolution
    for (size_t i = 0; i < targets.size(); i++) {
      if (targets[i]->frame && targets[i]->frame->size()) {
        union_grid->insert(*targets[i]->frame, deltas[i].inverse());
      }
    }
    return union_grid->calc_overlap_rate(*source, Eigen::Isometry3d::Identity());
  };

  // Check if new frame overlaps enough with existing keyframes
  {
    double max_overlap = 0.0;
    for (const auto& keyframe : keyframes) {
      const Eigen::Isometry3d delta = keyframe->T_world_imu.inverse() * frames[current]->T_world_imu;
      const double ov = fast_overlap(keyframe, frames[current]->frame, delta);
      max_overlap = std::max(max_overlap, ov);
    }
    if (max_overlap > params->epcd_keyframe_max_overlap) {
      return;
    }
  }

  const auto& new_keyframe = frames[current];
  keyframes.push_back(new_keyframe);

  if (keyframes.size() <= static_cast<size_t>(params->epcd_max_num_keyframes)) {
    return;
  }

  std::vector<EstimationFrame::ConstPtr> marginalized_keyframes;
  for (int i = 0; i < static_cast<int>(keyframes.size()); i++) {
    const Eigen::Isometry3d delta = keyframes[i]->T_world_imu.inverse() * new_keyframe->T_world_imu;
    const double overlap = fast_overlap(keyframes[i], new_keyframe->frame, delta);
    if (overlap < params->epcd_keyframe_min_overlap) {
      marginalized_keyframes.push_back(keyframes[i]);
      keyframes.erase(keyframes.begin() + i);
      i--;
    }
  }

  if (keyframes.size() <= static_cast<size_t>(params->epcd_max_num_keyframes)) {
    Callbacks::on_marginalized_keyframes(marginalized_keyframes);
    return;
  }

  std::vector<double> scores(keyframes.size() - 1, 0.0);
  for (int i = 0; i < static_cast<int>(keyframes.size()) - 1; i++) {
    const Eigen::Isometry3d delta_latest = keyframes[i]->T_world_imu.inverse() * new_keyframe->T_world_imu;
    const double overlap_latest = fast_overlap(keyframes[i], new_keyframe->frame, delta_latest);

    std::vector<EstimationFrame::ConstPtr> other_keyframes;
    std::vector<Eigen::Isometry3d> delta_from_others;
    for (int j = 0; j < static_cast<int>(keyframes.size()) - 1; j++) {
      if (i == j) continue;
      other_keyframes.push_back(keyframes[j]);
      delta_from_others.push_back(keyframes[j]->T_world_imu.inverse() * keyframes[i]->T_world_imu);
    }

    const double overlap_others = other_keyframes.empty() ? 0.0 : fast_overlap_multi(other_keyframes, keyframes[i]->frame, delta_from_others);
    scores[i] = overlap_latest * (1.0 - overlap_others);
  }

  const auto min_score = std::min_element(scores.begin(), scores.end());
  if (min_score != scores.end()) {
    const int frame_to_eliminate = std::distance(scores.begin(), min_score);
    marginalized_keyframes.push_back(keyframes[frame_to_eliminate]);
    keyframes.erase(keyframes.begin() + frame_to_eliminate);
  }
  Callbacks::on_marginalized_keyframes(marginalized_keyframes);
}

void OdometryEstimationCPU::update_keyframes_displacement(int current) {
  const auto params = static_cast<const OdometryEstimationCPUParams*>(this->params.get());

  if (keyframes.empty()) {
    keyframes.push_back(frames[current]);
    return;
  }

  const Eigen::Isometry3d delta_from_last = keyframes.back()->T_world_imu.inverse() * frames[current]->T_world_imu;
  const double delta_trans = delta_from_last.translation().norm();
  const double delta_rot = Eigen::AngleAxisd(delta_from_last.linear()).angle();

  if (delta_trans < params->epcd_keyframe_delta_trans && delta_rot < params->epcd_keyframe_delta_rot) {
    return;
  }

  const auto& new_keyframe = frames[current];
  keyframes.push_back(new_keyframe);

  if (keyframes.size() <= static_cast<size_t>(params->epcd_max_num_keyframes)) {
    return;
  }

  // Helper: get or fallback overlap using FastOccupancyGrid
  const auto fast_overlap_disp = [](const EstimationFrame::ConstPtr& target, const gtsam_points::PointCloud::ConstPtr& source, const Eigen::Isometry3d& delta) -> double {
    auto occ = target->get_custom_data<gtsam_points::FastOccupancyGrid>("occ_grid");
    if (occ) {
      return occ->calc_overlap_rate(*source, delta);
    }
    if (!target->voxelmaps.empty()) {
      return gtsam_points::overlap_auto(target->voxelmaps.back(), source, delta);
    }
    return 0.0;
  };

  std::vector<EstimationFrame::ConstPtr> marginalized_keyframes;
  for (int i = 0; i < static_cast<int>(keyframes.size()) - 1; i++) {
    const Eigen::Isometry3d delta = keyframes[i]->T_world_imu.inverse() * new_keyframe->T_world_imu;
    const double overlap = fast_overlap_disp(keyframes[i], new_keyframe->frame, delta);
    if (overlap < 0.01) {
      marginalized_keyframes.push_back(keyframes[i]);
      keyframes.erase(keyframes.begin() + i);
      Callbacks::on_marginalized_keyframes(marginalized_keyframes);
      return;
    }
  }

  const int leave_window = 2;
  const double eps = 1e-3;
  std::vector<double> scores(keyframes.size() - 1, 0.0);
  for (int i = leave_window; i < static_cast<int>(keyframes.size()) - 1; i++) {
    double sum_inv_dist = 0.0;
    for (int j = 0; j < static_cast<int>(keyframes.size()) - 1; j++) {
      if (i == j) {
        continue;
      }

      const double dist = (keyframes[i]->T_world_imu.translation() - keyframes[j]->T_world_imu.translation()).norm();
      sum_inv_dist += 1.0 / (dist + eps);
    }

    const double d0 = (keyframes[i]->T_world_imu.translation() - new_keyframe->T_world_imu.translation()).norm();
    scores[i] = std::sqrt(d0) * sum_inv_dist;
  }

  const auto max_score = std::max_element(scores.begin(), scores.end());
  if (max_score != scores.end()) {
    const int max_score_index = std::distance(scores.begin(), max_score);
    marginalized_keyframes.push_back(keyframes[max_score_index]);
    keyframes.erase(keyframes.begin() + max_score_index);
    Callbacks::on_marginalized_keyframes(marginalized_keyframes);
  }
}

void OdometryEstimationCPU::update_target(const int current, const Eigen::Isometry3d& T_target_imu) {
  const auto params = static_cast<const OdometryEstimationCPUParams*>(this->params.get());
  auto frame = frames[current]->frame;
  if (current >= 5) {
    frame = gtsam_points::random_sampling(frames[current]->frame, params->target_downsampling_rate, mt);
  }

  auto transformed = gtsam_points::transform(frame, T_target_imu);
  if (params->registration_type == "GICP") {
    target_ivox->insert(*transformed);
  } else if (params->registration_type == "VGICP") {
    for (auto& target_voxelmap : target_voxelmaps) {
      target_voxelmap->insert(*transformed);
    }
  }

  // Update target point cloud (just for visualization).
  // This is not necessary for mapping and can safely be removed.
  if (frames.size() % 50 == 0) {
    EstimationFrame::Ptr frame(new EstimationFrame);
    frame->id = frames.size() - 1;
    frame->stamp = frames.back()->stamp;

    frame->T_lidar_imu = frames.back()->T_lidar_imu;
    frame->T_world_lidar = frame->T_lidar_imu.inverse();
    frame->T_world_imu.setIdentity();

    frame->v_world_imu.setZero();
    frame->imu_bias.setZero();

    frame->frame_id = FrameID::IMU;

    if (params->registration_type == "GICP") {
      frame->frame = std::make_shared<gtsam_points::PointCloudCPU>(target_ivox->voxel_points());
    } else if (params->registration_type == "VGICP") {
      frame->frame = std::make_shared<gtsam_points::PointCloudCPU>(target_voxelmaps[0]->voxel_points());
    }

    std::vector<EstimationFrame::ConstPtr> keyframes = {frame};
    Callbacks::on_update_keyframes(keyframes);

    if (target_ivox_frame) {
      std::vector<EstimationFrame::ConstPtr> marginalized_keyframes = {target_ivox_frame};
      Callbacks::on_marginalized_keyframes(marginalized_keyframes);
    }

    target_ivox_frame = frame;
  }
}

}  // namespace glim
