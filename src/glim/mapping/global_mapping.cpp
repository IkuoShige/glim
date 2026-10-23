#include <glim/mapping/global_mapping.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <unordered_set>
#include <Eigen/Eigenvalues>
#include <spdlog/spdlog.h>
#include <boost/filesystem.hpp>

#include <gtsam/base/serialization.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PoseRotationPrior.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_points/config.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/factors/rotate_vector3_factor.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/optimizers/isam2_ext.hpp>
#include <gtsam_points/optimizers/isam2_ext_dummy.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>

#include <gtsam_points/ann/kdtree2.hpp>
#include <gtsam_points/features/covariance_estimation.hpp>
#include <gtsam_points/ann/fast_occupancy_grid.hpp>
#include <glim/factors/exact_gicp_factor.hpp>
#include <glim/util/config.hpp>
#include <glim/util/serialization.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/mapping/callbacks.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/task_arena.h>
#endif

namespace glim {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::E;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

using Callbacks = GlobalMappingCallbacks;

GlobalMappingParams::GlobalMappingParams() {
  Config config(GlobalConfig::get_config_path("config_global_mapping"));

  enable_imu = config.param<bool>("global_mapping", "enable_imu", true);
  enable_optimization = config.param<bool>("global_mapping", "enable_optimization", true);

  enable_between_factors = config.param<bool>("global_mapping", "create_between_factors", false);
  between_registration_type = config.param<std::string>("global_mapping", "between_registration_type", "GICP");
  registration_error_factor_type = config.param<std::string>("global_mapping", "registration_error_factor_type", "VGICP");
  submap_voxel_resolution = config.param<double>("global_mapping", "submap_voxel_resolution", 1.0);
  submap_voxel_resolution_max = config.param<double>("global_mapping", "submap_voxel_resolution_max", submap_voxel_resolution);
  submap_voxel_resolution_dmin = config.param<double>("global_mapping", "submap_voxel_resolution_dmin", 5.0);
  submap_voxel_resolution_dmax = config.param<double>("global_mapping", "submap_voxel_resolution_dmax", 20.0);

  submap_voxelmap_levels = config.param<int>("global_mapping", "submap_voxelmap_levels", 2);
  submap_voxelmap_scaling_factor = config.param<double>("global_mapping", "submap_voxelmap_scaling_factor", 2.0);
  if (registration_error_factor_type == "EPCD_GICP") {
    epcd = ExactGICPFactorParams::load(config, "global_mapping");
    epcd.num_threads = config.param<int>("global_mapping", "epcd_num_threads", epcd.num_threads);
  }

  randomsampling_rate = config.param<double>("global_mapping", "randomsampling_rate", 1.0);
  max_implicit_loop_distance = config.param<double>("global_mapping", "max_implicit_loop_distance", 100.0);
  min_implicit_loop_overlap = config.param<double>("global_mapping", "min_implicit_loop_overlap", 0.1);
  enable_loop_verification = config.param<bool>("global_mapping", "enable_loop_verification", false);
  loop_coarse_levels = std::max(1, config.param<int>("global_mapping", "loop_coarse_levels", 4));
  loop_consistency_submaps = std::max(1, config.param<int>("global_mapping", "loop_consistency_submaps", 2));

  enable_gpu = registration_error_factor_type.find("GPU") != std::string::npos;

  use_isam2_dogleg = config.param<bool>("global_mapping", "use_isam2_dogleg", false);
  isam2_relinearize_skip = config.param<int>("global_mapping", "isam2_relinearize_skip", 1);
  isam2_relinearize_thresh = config.param<double>("global_mapping", "isam2_relinearize_thresh", 0.1);

  init_pose_damping_scale = config.param<double>("global_mapping", "init_pose_damping_scale", 1e10);
}

GlobalMappingParams::~GlobalMappingParams() {}

GlobalMapping::GlobalMapping(const GlobalMappingParams& params) : params(params) {
#ifndef GTSAM_POINTS_USE_CUDA
  // Without CUDA, GPU factors would be silently dropped (e.g., offline_viewer with the default GPU config), so stop instead of building a map without them
  if (params.enable_gpu) {
    const std::string msg = "global_mapping: " + params.registration_error_factor_type + " requires GLIM built with GPU option; pass a CPU config (e.g., --config_path <glim>/config/epcd_livox_budget)";
    logger->critical(msg);
    throw std::runtime_error(msg);
  }
#endif

  session_id = 0;
  imu_integration.reset(new IMUIntegration);

  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);

  gtsam::ISAM2Params isam2_params;
  if (params.use_isam2_dogleg) {
    gtsam::ISAM2DoglegParams dogleg_params;
    isam2_params.setOptimizationParams(dogleg_params);
  }
  isam2_params.relinearizeSkip = params.isam2_relinearize_skip;
  isam2_params.setRelinearizeThreshold(params.isam2_relinearize_thresh);

  if (params.enable_optimization) {
    isam2.reset(new gtsam_points::ISAM2Ext(isam2_params));
  } else {
    isam2.reset(new gtsam_points::ISAM2ExtDummy(isam2_params));
  }

#ifdef GTSAM_POINTS_USE_CUDA
  stream_buffer_roundrobin = std::make_shared<gtsam_points::StreamTempBufferRoundRobin>(64);
#endif

#ifdef GTSAM_USE_TBB
  tbb_task_arena = std::make_shared<tbb::task_arena>(1);
#endif
}

GlobalMapping::~GlobalMapping() {}

void GlobalMapping::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);
  if (params.enable_imu) {
    imu_integration->insert_imu(stamp, linear_acc, angular_vel);
  }
}

void GlobalMapping::insert_submap(const SubMap::Ptr& submap) {
  logger->debug("insert_submap id={} |frame|={}", submap->id, submap->frame->size());

  const int current = submaps.size();
  const int last = current - 1;
  insert_submap(current, submap);

  gtsam::Pose3 current_T_world_submap = gtsam::Pose3::Identity();
  gtsam::Pose3 last_T_world_submap = gtsam::Pose3::Identity();

  if (current != 0) {
    if (isam2->valueExists(X(last))) {
      last_T_world_submap = isam2->calculateEstimate<gtsam::Pose3>(X(last));
    } else {
      last_T_world_submap = new_values->at<gtsam::Pose3>(X(last));
    }

    const Eigen::Isometry3d T_origin0_endpointR0 = submaps[last]->T_origin_endpoint_R;
    const Eigen::Isometry3d T_origin1_endpointL1 = submaps[current]->T_origin_endpoint_L;
    const Eigen::Isometry3d T_endpointR0_endpointL1 = submaps[last]->odom_frames.back()->T_world_sensor().inverse() * submaps[current]->odom_frames.front()->T_world_sensor();
    const Eigen::Isometry3d T_origin0_origin1 = T_origin0_endpointR0 * T_endpointR0_endpointL1 * T_origin1_endpointL1.inverse();

    current_T_world_submap = last_T_world_submap * gtsam::Pose3(T_origin0_origin1.matrix());
  } else {
    current_T_world_submap = gtsam::Pose3(submap->T_world_origin.matrix());
  }

  new_values->insert(X(current), current_T_world_submap);
  submap->T_world_origin = Eigen::Isometry3d(current_T_world_submap.matrix());

  Callbacks::on_insert_submap(submap);

  submap->drop_frame_points();

  if (current == 0) {
    new_factors->emplace_shared<gtsam_points::LinearDampingFactor>(X(0), 6, params.init_pose_damping_scale);
  } else {
    new_factors->add(*create_between_factors(current));
    new_factors->add(*create_matching_cost_factors(current));
  }

  if (params.enable_imu) {
    logger->debug("create IMU factor");
    if (submap->odom_frames.front()->frame_id != FrameID::IMU) {
      logger->warn("odom frames are not estimated in the IMU frame while global mapping requires IMU estimation");
    }

    // Local velocities
    const gtsam::imuBias::ConstantBias imu_biasL(submap->frames.front()->imu_bias);
    const gtsam::imuBias::ConstantBias imu_biasR(submap->frames.back()->imu_bias);

    const Eigen::Vector3d v_origin_imuL = submap->T_world_origin.linear().inverse() * submap->frames.front()->v_world_imu;
    const Eigen::Vector3d v_origin_imuR = submap->T_world_origin.linear().inverse() * submap->frames.back()->v_world_imu;

    const auto prior_noise3 = gtsam::noiseModel::Isotropic::Precision(3, 1e6);
    const auto prior_noise6 = gtsam::noiseModel::Isotropic::Precision(6, 1e6);

    if (current > 0) {
      new_values->insert(E(current * 2), gtsam::Pose3((submap->T_world_origin * submap->T_origin_endpoint_L).matrix()));
      new_values->insert(V(current * 2), (submap->T_world_origin.linear() * v_origin_imuL).eval());
      new_values->insert(B(current * 2), imu_biasL);

      new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(current), E(current * 2), gtsam::Pose3(submap->T_origin_endpoint_L.matrix()), prior_noise6);
      new_factors->emplace_shared<gtsam_points::RotateVector3Factor>(X(current), V(current * 2), v_origin_imuL, prior_noise3);
      new_factors->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(current * 2), imu_biasL, prior_noise6);
      new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(current * 2), B(current * 2 + 1), gtsam::imuBias::ConstantBias(), prior_noise6);
    }

    new_values->insert(E(current * 2 + 1), gtsam::Pose3((submap->T_world_origin * submap->T_origin_endpoint_R).matrix()));
    new_values->insert(V(current * 2 + 1), (submap->T_world_origin.linear() * v_origin_imuR).eval());
    new_values->insert(B(current * 2 + 1), imu_biasR);

    new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(current), E(current * 2 + 1), gtsam::Pose3(submap->T_origin_endpoint_R.matrix()), prior_noise6);
    new_factors->emplace_shared<gtsam_points::RotateVector3Factor>(X(current), V(current * 2 + 1), v_origin_imuR, prior_noise3);
    new_factors->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(current * 2 + 1), imu_biasR, prior_noise6);

    if (current != 0) {
      const double stampL = submaps[last]->frames.back()->stamp;
      const double stampR = submaps[current]->frames.front()->stamp;

      int num_integrated;
      const int imu_read_cursor = imu_integration->integrate_imu(stampL, stampR, imu_biasL, &num_integrated);
      imu_integration->erase_imu_data(imu_read_cursor);

      if (num_integrated < 2) {
        logger->warn("insufficient IMU data between submaps (global_mapping)!!");
        new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(V(last * 2 + 1), V(current * 2), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Precision(3, 1.0));
      } else {
        new_factors
          ->emplace_shared<gtsam::ImuFactor>(E(last * 2 + 1), V(last * 2 + 1), E(current * 2), V(current * 2), B(last * 2 + 1), imu_integration->integrated_measurements());
      }
    }
  }

  logger->debug("|new_factors|={} |new_values|={}", new_factors->size(), new_values->size());

  Callbacks::on_smoother_update(*isam2, *new_factors, *new_values);
  auto result = update_isam2(*new_factors, *new_values, factors_to_remove_);
  Callbacks::on_smoother_update_result(*isam2, result);

  // Remember the isam2 indices of loop corrections to replace them later
  for (auto& correction : new_loop_corrections_) {
    const size_t k = std::find(new_factors->begin(), new_factors->end(), correction.factor) - new_factors->begin();
    if (k < result.newFactorsIndices.size()) {
      correction.isam2_index = result.newFactorsIndices[k];
      loop_corrections_.push_back(correction);
    }
  }
  new_loop_corrections_.clear();
  factors_to_remove_.clear();

  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);

  update_submaps();
  Callbacks::on_update_submaps(submaps);
}

void GlobalMapping::insert_submap(int current, const SubMap::Ptr& submap) {
  submap->voxelmaps.clear();

  // Adaptively determine the voxel resolution based on the median distance
  const int max_scan_count = 256;
  const double dist_median = gtsam_points::median_distance(submap->frame, max_scan_count);
  const double p = std::max(0.0, std::min(1.0, (dist_median - params.submap_voxel_resolution_dmin) / (params.submap_voxel_resolution_dmax - params.submap_voxel_resolution_dmin)));
  const double base_resolution = params.submap_voxel_resolution + p * (params.submap_voxel_resolution_max - params.submap_voxel_resolution);

  // Create frame and voxelmaps
  gtsam_points::PointCloud::ConstPtr subsampled_submap;
  if (params.randomsampling_rate > 0.99) {
    subsampled_submap = submap->frame;
  } else {
    subsampled_submap = gtsam_points::random_sampling(submap->frame, params.randomsampling_rate, mt);
  }

#ifdef GTSAM_POINTS_USE_CUDA
  if (params.enable_gpu && !submap->frame->points_gpu) {
    submap->frame = gtsam_points::PointCloudGPU::clone(*submap->frame);
  }

  if (params.enable_gpu) {
    if (params.randomsampling_rate > 0.99) {
      subsampled_submap = submap->frame;
    } else {
      subsampled_submap = gtsam_points::PointCloudGPU::clone(*subsampled_submap);
    }

    for (int i = 0; i < params.submap_voxelmap_levels; i++) {
      const double resolution = base_resolution * std::pow(params.submap_voxelmap_scaling_factor, i);
      auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(resolution);
      voxelmap->insert(*submap->frame);
      submap->voxelmaps.push_back(voxelmap);
    }
  }
#endif

  // EPCD factors register against the submap points, so voxelmaps would only cost memory
  if (submap->voxelmaps.empty() && params.registration_error_factor_type != "EPCD_GICP") {
    for (int i = 0; i < params.submap_voxelmap_levels; i++) {
      const double resolution = base_resolution * std::pow(params.submap_voxelmap_scaling_factor, i);
      auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
      voxelmap->insert(*subsampled_submap);
      submap->voxelmaps.push_back(voxelmap);
    }
  }

  occ_grids_.push_back(create_occupancy_grid(*subsampled_submap, base_resolution));
  submaps.push_back(submap);
  subsampled_submaps.push_back(subsampled_submap);
}

void GlobalMapping::find_overlapping_submaps(double min_overlap) {
  if (submaps.empty()) {
    return;
  }

  // Between factors are Vector2i actually. A bad use of Vector3i
  std::unordered_set<Eigen::Vector3i, gtsam_points::Vector3iHash> existing_factors;
  for (const auto& factor : isam2->getFactorsUnsafe()) {
    if (factor == nullptr) {
      continue;
    }
    if (factor->keys().size() != 2) {
      continue;
    }

    gtsam::Symbol sym1(factor->keys()[0]);
    gtsam::Symbol sym2(factor->keys()[1]);
    if (sym1.chr() != 'x' || sym2.chr() != 'x') {
      continue;
    }

    existing_factors.emplace(sym1.index(), sym2.index(), 0);
  }

  double squared_max_implicit_loop_distance = params.max_implicit_loop_distance * params.max_implicit_loop_distance;
  for (int i = 0; i < submaps.size(); i++) {
    for (int j = i + 1; j < submaps.size(); j++) {
      if (existing_factors.count(Eigen::Vector3i(i, j, 0))) {
        continue;
      }

      const Eigen::Isometry3d delta = submaps[i]->T_world_origin.inverse() * submaps[j]->T_world_origin;
      const double squared_dist = delta.translation().squaredNorm();
      if (squared_dist > squared_max_implicit_loop_distance) {
        continue;
      }

      const double overlap = occ_grids_[i]->calc_overlap_rate(*subsampled_submaps[j], delta);
      if (overlap < min_overlap) {
        continue;
      }

      new_factors->add(*create_registration_factors(i, j));
    }
  }

  logger->info("new overlapping {} submap pairs found", new_factors->size());

  Callbacks::on_smoother_update(*isam2, *new_factors, *new_values);
  auto result = update_isam2(*new_factors, *new_values);
  Callbacks::on_smoother_update_result(*isam2, result);

  new_factors->resize(0);
  new_values->clear();

  update_submaps();
  Callbacks::on_update_submaps(submaps);
}

void GlobalMapping::optimize() {
  if (isam2->empty()) {
    return;
  }

  logger->debug("|new_factors|={} |new_values|={}", new_factors->size(), new_values->size());

  Callbacks::on_smoother_update(*isam2, *new_factors, *new_values);
  auto result = update_isam2(*new_factors, *new_values);

  new_factors.reset(new gtsam::NonlinearFactorGraph);
  new_values.reset(new gtsam::Values);

  Callbacks::on_smoother_update_result(*isam2, result);

  update_submaps();
  Callbacks::on_update_submaps(submaps);
}

std::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMapping::create_between_factors(int current) const {
  auto factors = std::make_shared<gtsam::NonlinearFactorGraph>();
  if (current == 0 || !params.enable_between_factors) {
    return factors;
  }

  const int last = current - 1;
  const gtsam::Pose3 init_delta = gtsam::Pose3((submaps[last]->T_world_origin.inverse() * submaps[current]->T_world_origin).matrix());

  if (params.between_registration_type == "NONE") {
    factors->add(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), init_delta, gtsam::noiseModel::Isotropic::Precision(6, 1e6)));
    return factors;
  }

  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3::Identity());
  values.insert(X(1), init_delta);

  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3::Identity(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));

  auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(X(0), X(1), submaps[last]->frame, submaps[current]->frame);
  factor->set_max_correspondence_distance(0.5);
  factor->set_num_threads(2);
  graph.add(factor);

  logger->debug("--- LM optimization ---");
  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setlambdaInitial(1e-12);
  lm_params.setMaxIterations(10);
  lm_params.callback = [this](const auto& status, const auto& values) { logger->debug(status.to_string()); };

#ifdef GTSAM_USE_TBB
  auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
  arena->execute([&] {
#endif
    gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
    values = optimizer.optimize();

#ifdef GTSAM_USE_TBB
  });
#endif

  const gtsam::Pose3 estimated_delta = values.at<gtsam::Pose3>(X(1));
  const auto linearized = factor->linearize(values);
  const gtsam::Matrix6 H = linearized->hessianBlockDiagonal()[X(1)] + 1e6 * gtsam::Matrix6::Identity();

  factors->add(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), estimated_delta, gtsam::noiseModel::Gaussian::Information(H)));
  return factors;
}

// Bit-packed occupancy grid at the coarsest voxelmap resolution for fast overlap estimation (paper Sec. III-B)
std::shared_ptr<gtsam_points::FastOccupancyGrid> GlobalMapping::create_occupancy_grid(const gtsam_points::PointCloud& points, double base_resolution) const {
  const double resolution = base_resolution * std::pow(params.submap_voxelmap_scaling_factor, params.submap_voxelmap_levels - 1);
  auto occ_grid = std::make_shared<gtsam_points::FastOccupancyGrid>(resolution);
  occ_grid->insert(points);
  return occ_grid;
}

gtsam::NonlinearFactor::shared_ptr GlobalMapping::create_epcd_factor(int target, int source) const {
  auto& target_tree = kdtree_cache_[target];
  if (!target_tree) {
    target_tree = std::make_shared<gtsam_points::KdTree2<gtsam_points::PointCloud>>(subsampled_submaps[target], params.epcd.num_threads);
  }
  auto factor = gtsam::make_shared<ExactGICPFactor>(X(target), X(source), subsampled_submaps[target], subsampled_submaps[source], target_tree);
  factor->set_params(params.epcd);
  return factor;
}

std::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMapping::create_registration_factors(int target, int source) const {
  auto factors = std::make_shared<gtsam::NonlinearFactorGraph>();
  if (params.registration_error_factor_type == "VGICP") {
    for (const auto& voxelmap : submaps[target]->voxelmaps) {
      factors->emplace_shared<gtsam_points::IntegratedVGICPFactor>(X(target), X(source), voxelmap, subsampled_submaps[source]);
    }
  } else if (params.registration_error_factor_type == "EPCD_GICP") {
    factors->add(create_epcd_factor(target, source));
  }
#ifdef GTSAM_POINTS_USE_CUDA
  else if (params.registration_error_factor_type == "VGICP_GPU") {
    const auto stream_buffer = std::any_cast<std::shared_ptr<gtsam_points::StreamTempBufferRoundRobin>>(stream_buffer_roundrobin)->get_stream_buffer();
    const auto& stream = stream_buffer.first;
    const auto& buffer = stream_buffer.second;
    for (const auto& voxelmap : submaps[target]->voxelmaps) {
      factors->emplace_shared<gtsam_points::IntegratedVGICPFactorGPU>(X(target), X(source), voxelmap, subsampled_submaps[source], stream, buffer);
    }
  }
#endif
  else {
    logger->warn("unknown registration error type ({})", params.registration_error_factor_type);
  }
  return factors;
}

gtsam_points::PointCloud::ConstPtr GlobalMapping::coarse_frame(int submap_id, double resolution) const {
  // Recomputed on demand because caching them for all submaps costs memory
  auto frame = gtsam_points::voxelgrid_sampling(submaps[submap_id]->frame, resolution, 2);
  frame->add_covs(gtsam_points::estimate_covariances(*frame, 10, 2));
  return frame;
}

double GlobalMapping::source_displacement(int source, const Eigen::Isometry3d& T_a, const Eigen::Isometry3d& T_b) const {
  // RMS displacement of the source points between two poses
  const auto& frame = subsampled_submaps[source];
  const int step = std::max<int>(1, frame->size() / 256);
  const Eigen::Isometry3d delta = T_a.inverse() * T_b;
  double sum_sq = 0.0;
  int count = 0;
  for (int k = 0; k < frame->size(); k += step) {
    const Eigen::Vector3d p = frame->points[k].head<3>();
    sum_sq += (delta * p - p).squaredNorm();
    count++;
  }
  return count ? std::sqrt(sum_sq / count) : 0.0;
}

GlobalMapping::LoopAlignment GlobalMapping::align_coarse_to_fine(int target, int source, const Eigen::Isometry3d& init_T_target_source) const {
  const double finest = params.epcd.max_correspondence_distance;
  gtsam::Pose3 T_target_source(init_T_target_source.matrix());

  auto optimize = [&](const gtsam_points::PointCloud::ConstPtr& target_frame, const gtsam_points::PointCloud::ConstPtr& source_frame, double max_corr) {
    gtsam::Values values;
    values.insert(X(0), gtsam::Pose3::Identity());
    values.insert(X(1), T_target_source);

    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3::Identity(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));
    auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(X(0), X(1), target_frame, source_frame);
    factor->set_max_correspondence_distance(max_corr);
    factor->set_num_threads(2);
    graph.add(factor);

    gtsam_points::LevenbergMarquardtExtParams lm_params;
    lm_params.setlambdaInitial(1e-12);
    lm_params.setMaxIterations(10);

#ifdef GTSAM_USE_TBB
    auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
    arena->execute([&] {
#endif
      gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
      values = optimizer.optimize();
#ifdef GTSAM_USE_TBB
    });
#endif
    return values.at<gtsam::Pose3>(X(1));
  };

  // Coarse-to-fine. Stop once a stage moves the source less than the finest correspondence distance
  for (int level = params.loop_coarse_levels - 1; level > 0; level--) {
    const double max_corr = finest * std::pow(2.0, level);
    const auto next = optimize(coarse_frame(target, max_corr / 4.0), coarse_frame(source, max_corr / 4.0), max_corr);
    const double moved = source_displacement(source, Eigen::Isometry3d(T_target_source.matrix()), Eigen::Isometry3d(next.matrix()));
    T_target_source = next;
    if (moved < finest) {
      break;
    }
  }
  T_target_source = optimize(submaps[target]->frame, submaps[source]->frame, finest);

  // Accept only if inliers do not decrease and all six DoFs are observable
  auto check = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(X(0), X(1), submaps[target]->frame, submaps[source]->frame);
  check->set_max_correspondence_distance(finest);
  check->set_num_threads(2);

  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3::Identity());
  values.insert(X(1), gtsam::Pose3(init_T_target_source.matrix()));
  check->error(values);
  const double inlier_before = check->inlier_fraction();

  values.update(X(1), T_target_source);
  check->error(values);
  const double inlier_after = check->inlier_fraction();

  LoopAlignment result;
  result.T_target_source = Eigen::Isometry3d(T_target_source.matrix());
  result.H = check->linearize(values)->hessianBlockDiagonal()[X(1)];

  Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> eig(result.H);
  const bool observable = eig.eigenvalues()[0] > 1e-6 * eig.eigenvalues()[5];
  result.valid = observable && inlier_after >= inlier_before;
  return result;
}

std::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMapping::confirm_loop(const LoopHypothesis& hypothesis) {
  auto factors = std::make_shared<gtsam::NonlinearFactorGraph>();
  if (!hypothesis.in_basin) {
    // Out of the basin: pull the estimate in with a between factor
    const gtsam::Pose3 measured(hypothesis.alignment.T_target_source.matrix());
    const auto noise = gtsam::noiseModel::Gaussian::Information(hypothesis.alignment.H);
    const auto between = gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(hypothesis.representative), X(hypothesis.source), measured, noise);
    factors->add(between);
    new_loop_corrections_.push_back(LoopCorrection{hypothesis.representative, hypothesis.source, between});
    logger->info("loop correction ({}, {}) inserted ({} target submaps)", hypothesis.representative, hypothesis.source, hypothesis.targets.size());
    return factors;
  }

  for (const int target : hypothesis.targets) {
    factors->add(*create_registration_factors(target, hypothesis.source));
  }

  // Replace loop corrections towards these submaps with registration error factors
  for (auto it = loop_corrections_.begin(); it != loop_corrections_.end();) {
    if (std::find(hypothesis.targets.begin(), hypothesis.targets.end(), it->target) == hypothesis.targets.end()) {
      it++;
      continue;
    }
    factors->add(*create_registration_factors(it->target, it->source));
    factors_to_remove_.push_back(it->isam2_index);
    logger->info("loop correction ({}, {}) replaced with registration error factors", it->target, it->source);
    it = loop_corrections_.erase(it);
  }
  return factors;
}

std::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMapping::create_verified_loop_factors(int current, const std::vector<std::pair<int, double>>& candidates) {
  // Registration error factors converge only within about the correspondence distance,
  // so each revisit (a run of consecutive overlapping submaps) is registered coarse-to-fine first
  auto factors = std::make_shared<gtsam::NonlinearFactorGraph>();
  const double basin = params.epcd.max_correspondence_distance;
  const Eigen::Isometry3d T_world_current = submaps[current]->T_world_origin;

  std::vector<std::vector<std::pair<int, double>>> groups;
  for (const auto& candidate : candidates) {
    if (groups.empty() || candidate.first != groups.back().back().first + 1) {
      groups.emplace_back();
    }
    groups.back().push_back(candidate);
  }

  std::vector<LoopHypothesis> hypotheses;
  for (auto& group : groups) {
    std::vector<int> targets;
    for (const auto& member : group) {
      targets.push_back(member.first);
    }

    // The submaps being traversed get registration error factors directly
    if (group.back().first == current - 1) {
      for (const int target : targets) {
        factors->add(*create_registration_factors(target, current));
      }
      continue;
    }

    std::sort(group.begin(), group.end(), [](const auto& lhs, const auto& rhs) { return lhs.second > rhs.second; });
    std::vector<Eigen::Isometry3d> implied;
    LoopHypothesis hypothesis;
    for (int k = 0; k < std::min<int>(2, group.size()); k++) {
      const int target = group[k].first;
      const auto alignment = align_coarse_to_fine(target, current, submaps[target]->T_world_origin.inverse() * T_world_current);
      if (!alignment.valid) {
        continue;
      }
      if (implied.empty()) {
        hypothesis.representative = target;
        hypothesis.alignment = alignment;
      }
      implied.push_back(submaps[target]->T_world_origin * alignment.T_target_source);
    }
    if (implied.empty()) {
      continue;
    }

    hypothesis.targets = targets;
    hypothesis.source = current;
    hypothesis.T_world_source = implied.front();
    hypothesis.in_basin = source_displacement(current, implied.front(), T_world_current) < basin;
    hypothesis.confirmations = (implied.size() == 2 && source_displacement(current, implied[0], implied[1]) < basin) ? 1 : 0;
    hypothesis.rejected = false;
    hypotheses.push_back(hypothesis);
  }

  // Temporal consistency: hypotheses on the same revisit must imply the estimated relative motion
  for (auto& pending : pending_loops_) {
    for (auto& hypothesis : hypotheses) {
      const bool same_revisit = std::any_of(pending.targets.begin(), pending.targets.end(), [&](int target) {
        return std::find(hypothesis.targets.begin(), hypothesis.targets.end(), target) != hypothesis.targets.end();
      });
      if (!same_revisit) {
        continue;
      }

      const Eigen::Isometry3d implied_motion = pending.T_world_source.inverse() * hypothesis.T_world_source;
      const Eigen::Isometry3d estimated_motion = submaps[pending.source]->T_world_origin.inverse() * T_world_current;
      if (source_displacement(current, implied_motion, estimated_motion) < basin) {
        pending.confirmations++;
        hypothesis.confirmations++;
      } else {
        pending.rejected = true;
      }
    }
  }

  std::vector<LoopHypothesis> remaining;
  for (auto& pending : pending_loops_) {
    if (pending.rejected) {
      logger->debug("loop hypothesis ({}, {}) rejected by temporal consistency", pending.representative, pending.source);
    } else if (pending.confirmations >= params.loop_consistency_submaps) {
      factors->add(*confirm_loop(pending));
    } else if (current - pending.source < params.loop_consistency_submaps) {
      remaining.push_back(pending);
    }
  }

  for (auto& hypothesis : hypotheses) {
    // Two agreeing registrations confirm an in-basin revisit immediately
    if (hypothesis.in_basin && hypothesis.confirmations >= 1) {
      factors->add(*confirm_loop(hypothesis));
    } else if (hypothesis.confirmations >= params.loop_consistency_submaps) {
      factors->add(*confirm_loop(hypothesis));
    } else {
      remaining.push_back(hypothesis);
    }
  }
  pending_loops_ = std::move(remaining);

  return factors;
}

std::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMapping::create_matching_cost_factors(int current) {
  auto factors = std::make_shared<gtsam::NonlinearFactorGraph>();
  if (current == 0) {
    return factors;
  }

  const auto& current_submap = submaps.back();

  double previous_overlap = 0.0;
  double squared_max_implicit_loop_distance = params.max_implicit_loop_distance * params.max_implicit_loop_distance;

  std::vector<std::pair<int, double>> candidates;
  for (int i = 0; i < current; i++) {
    const double squared_dist = (submaps[i]->T_world_origin.translation() - current_submap->T_world_origin.translation()).squaredNorm();
    if (squared_dist > squared_max_implicit_loop_distance) {
      continue;
    }

    const Eigen::Isometry3d delta = submaps[i]->T_world_origin.inverse() * current_submap->T_world_origin;
    const double overlap = occ_grids_[i]->calc_overlap_rate(*current_submap->frame, delta);

    previous_overlap = i == current - 1 ? overlap : previous_overlap;
    if (overlap < params.min_implicit_loop_overlap) {
      continue;
    }

    candidates.emplace_back(i, overlap);
  }

  if (params.enable_loop_verification) {
    factors->add(*create_verified_loop_factors(current, candidates));
  } else {
    for (const auto& candidate : candidates) {
      factors->add(*create_registration_factors(candidate.first, current));
    }
  }

  if (previous_overlap < std::max(0.25, params.min_implicit_loop_overlap)) {
    logger->warn("previous submap has only a small overlap with the current submap ({})", previous_overlap);
    logger->warn("create a between factor to prevent the submap from being isolated");
    const int last = current - 1;
    const gtsam::Pose3 init_delta = gtsam::Pose3((submaps[last]->T_world_origin.inverse() * submaps[current]->T_world_origin).matrix());
    factors->add(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), init_delta, gtsam::noiseModel::Isotropic::Precision(6, 1e6)));
  }

  return factors;
}

void GlobalMapping::update_submaps() {
  for (int i = 0; i < submaps.size(); i++) {
    submaps[i]->T_world_origin = Eigen::Isometry3d(isam2->calculateEstimate<gtsam::Pose3>(X(i)).matrix());
  }
}

gtsam_points::ISAM2ResultExt GlobalMapping::update_isam2(const gtsam::NonlinearFactorGraph& new_factors, const gtsam::Values& new_values, const gtsam::FactorIndices& remove_indices) {
  gtsam::NonlinearFactorGraph factors = new_factors;
  gtsam::Values values = new_values;
  gtsam::FactorIndices removes = remove_indices;
  std::unordered_set<gtsam::Key> damped_keys;

  while (true) {
    gtsam::Key indeterminant_nearby_key = 0;
    try {
      gtsam_points::ISAM2ResultExt result;
#ifdef GTSAM_USE_TBB
      auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
      arena->execute([&] {
#endif
        result = isam2->update(factors, values, removes);
#ifdef GTSAM_USE_TBB
      });
#endif
      return result;
    } catch (const gtsam::IndeterminantLinearSystemException& e) {
      logger->error("an indeterminant linear system exception was caught during global map optimization!!");
      logger->error(e.what());
      indeterminant_nearby_key = e.nearbyVariable();
    } catch (const std::exception& e) {
      logger->error("an exception was caught during global map optimization!!");
      logger->error(e.what());
      return gtsam_points::ISAM2ResultExt();
    }

    const gtsam::Symbol symbol(indeterminant_nearby_key);
    if (symbol.chr() == 'v' || symbol.chr() == 'b' || symbol.chr() == 'e') {
      indeterminant_nearby_key = X(symbol.index() / 2);
    }

    // Damping fixes an underconstrained variable. If the same variable is reported again after it was damped,
    // the system is not underconstrained but numerically broken (e.g., the estimate has diverged), and rebuilding
    // the graph again would repeat forever
    const bool damping_failed = !damped_keys.insert(indeterminant_nearby_key).second;
    if (damping_failed) {
      logger->critical("the damping factor at {} did not resolve the indeterminant system", std::string(gtsam::Symbol(indeterminant_nearby_key)));
      logger->critical("global map optimization is disabled for the rest of this session");
      params.enable_optimization = false;
    } else {
      logger->warn("insert a damping factor at {} to prevent corruption", std::string(gtsam::Symbol(indeterminant_nearby_key)));
    }

    values = isam2->getLinearizationPoint();
    factors = isam2->getFactorsUnsafe();
    removes.clear();
    if (!damping_failed) {
      factors.emplace_shared<gtsam_points::LinearDampingFactor>(indeterminant_nearby_key, 6, 1e3);
    }

    gtsam::ISAM2Params isam2_params;
    if (params.use_isam2_dogleg) {
      gtsam::ISAM2DoglegParams dogleg_params;
      isam2_params.setOptimizationParams(dogleg_params);
    }
    isam2_params.relinearizeSkip = params.isam2_relinearize_skip;
    isam2_params.setRelinearizeThreshold(params.isam2_relinearize_thresh);

    if (params.enable_optimization) {
      isam2.reset(new gtsam_points::ISAM2Ext(isam2_params));
    } else {
      isam2.reset(new gtsam_points::ISAM2ExtDummy(isam2_params));
    }

    logger->warn("reset isam2");
  }
}

void GlobalMapping::save(const std::string& path) {
  optimize();

  boost::filesystem::create_directories(path);

  gtsam::NonlinearFactorGraph serializable_factors;
  std::unordered_map<std::string, gtsam::NonlinearFactor::shared_ptr> matching_cost_factors;

  for (const auto& factor : isam2->getFactorsUnsafe()) {
    bool serializable = !dynamic_cast<gtsam_points::IntegratedMatchingCostFactor*>(factor.get())
#ifdef GTSAM_POINTS_USE_CUDA
                        && !dynamic_cast<gtsam_points::IntegratedVGICPFactorGPU*>(factor.get())
#endif
      ;

    if (serializable) {
      serializable_factors.push_back(factor);
    } else {
      const gtsam::Symbol symbol0(factor->keys()[0]);
      const gtsam::Symbol symbol1(factor->keys()[1]);
      const std::string key = std::to_string(symbol0.index()) + "_" + std::to_string(symbol1.index());

      matching_cost_factors[key] = factor;
    }
  }

  logger->info("serializing factor graph to {}/graph.bin", path);
  serializeToBinaryFile(serializable_factors, path + "/graph.bin");
  serializeToBinaryFile(isam2->calculateEstimate(), path + "/values.bin");

  std::ofstream ofs(path + "/graph.txt");
  ofs << "num_submaps: " << submaps.size() << std::endl;
  ofs << "num_all_frames: " << std::accumulate(submaps.begin(), submaps.end(), 0, [](int sum, const SubMap::ConstPtr& submap) { return sum + submap->frames.size(); }) << std::endl;

  ofs << "num_matching_cost_factors: " << matching_cost_factors.size() << std::endl;
  for (const auto& factor : matching_cost_factors) {
    std::string type;

    if (dynamic_cast<gtsam_points::IntegratedGICPFactor*>(factor.second.get())) {
      type = "gicp";
    } else if (dynamic_cast<ExactGICPFactor*>(factor.second.get())) {
      type = "epcd_gicp";
    } else if (dynamic_cast<gtsam_points::IntegratedVGICPFactor*>(factor.second.get())) {
      type = "vgicp";
    }
#ifdef GTSAM_POINTS_USE_CUDA
    else if (dynamic_cast<gtsam_points::IntegratedVGICPFactorGPU*>(factor.second.get())) {
      type = "vgicp_gpu";
    }
#endif

    gtsam::Symbol symbol0(factor.second->keys()[0]);
    gtsam::Symbol symbol1(factor.second->keys()[1]);
    ofs << "matching_cost " << type << " " << symbol0.index() << " " << symbol1.index() << std::endl;
  }

  std::ofstream odom_lidar_ofs(path + "/odom_lidar.txt");
  std::ofstream traj_lidar_ofs(path + "/traj_lidar.txt");

  std::ofstream odom_imu_ofs(path + "/odom_imu.txt");
  std::ofstream traj_imu_ofs(path + "/traj_imu.txt");

  const auto write_tum_frame = [](std::ofstream& ofs, const double stamp, const Eigen::Isometry3d& pose) {
    const Eigen::Quaterniond quat(pose.linear());
    const Eigen::Vector3d trans(pose.translation());
    ofs << boost::format("%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f") % stamp % trans.x() % trans.y() % trans.z() % quat.x() % quat.y() % quat.z() % quat.w() << std::endl;
  };

  for (int i = 0; i < submaps.size(); i++) {
    for (const auto& frame : submaps[i]->odom_frames) {
      write_tum_frame(odom_lidar_ofs, frame->stamp, frame->T_world_lidar);
      write_tum_frame(odom_imu_ofs, frame->stamp, frame->T_world_imu);
    }

    const Eigen::Isometry3d T_world_endpoint_L = submaps[i]->T_world_origin * submaps[i]->T_origin_endpoint_L;
    const Eigen::Isometry3d T_odom_lidar0 = submaps[i]->frames.front()->T_world_lidar;
    const Eigen::Isometry3d T_odom_imu0 = submaps[i]->frames.front()->T_world_imu;

    for (const auto& frame : submaps[i]->frames) {
      const Eigen::Isometry3d T_world_imu = T_world_endpoint_L * T_odom_imu0.inverse() * frame->T_world_imu;
      const Eigen::Isometry3d T_world_lidar = T_world_imu * frame->T_lidar_imu.inverse();

      write_tum_frame(traj_imu_ofs, frame->stamp, T_world_imu);
      write_tum_frame(traj_lidar_ofs, frame->stamp, T_world_lidar);
    }

    submaps[i]->save((boost::format("%s/%06d") % path % i).str());
  }

  logger->info("saving config");
  GlobalConfig::instance()->dump(path + "/config");
}


gtsam_points::PointCloud::Ptr GlobalMapping::export_points() {
  auto merged = std::make_shared<gtsam_points::PointCloudCPU>();

  size_t total_points = 0;
  for (const auto& submap : submaps) {
    if (!submap || !submap->frame) {
      continue;
    }
    total_points += submap->frame->size();
  }

  std::vector<Eigen::Vector4d> points;
  points.reserve(total_points);

  bool export_intensities = true;
  for (const auto& submap : submaps) {
    if (!submap || !submap->frame || !submap->frame->has_intensities()) {
      export_intensities = false;
      break;
    }
  }

  std::vector<double> intensities;
  if (export_intensities) {
    intensities.reserve(total_points);
  }

  for (const auto& submap : submaps) {
    if (!submap || !submap->frame) {
      continue;
    }

    for (int i = 0; i < submap->frame->size(); i++) {
      const Eigen::Vector4d point = submap->T_world_origin * submap->frame->points[i];
      points.push_back(point);

      if (export_intensities) {
        intensities.push_back(submap->frame->intensities[i]);
      }
    }
  }

  if (!points.empty()) {
    merged->add_points(points);
    if (export_intensities) {
      merged->add_intensities(intensities);
    }
  }

  return merged;
}

bool GlobalMapping::load(const std::string& path) {
  std::ifstream ifs(path + "/graph.txt");
  if (!ifs) {
    logger->error("failed to open {}/graph.txt", path);
    return false;
  }

  const int start_from_frame_id = submaps.size();

  std::string token;
  int num_submaps, num_all_frames, num_matching_cost_factors;

  ifs >> token >> num_submaps;
  ifs >> token >> num_all_frames;
  ifs >> token >> num_matching_cost_factors;

  std::vector<std::tuple<std::string, int, int>> matching_cost_factors(num_matching_cost_factors);
  for (int i = 0; i < num_matching_cost_factors; i++) {
    auto& factor = matching_cost_factors[i];
    ifs >> token >> std::get<0>(factor) >> std::get<1>(factor) >> std::get<2>(factor);
  }

  logger->info("Load submaps (session_id={})", session_id);
  submaps.reserve(submaps.size() + num_submaps);
  subsampled_submaps.reserve(submaps.size() + num_submaps);
  for (int i = 0; i < num_submaps; i++) {
    auto submap = SubMap::load((boost::format("%s/%06d") % path % i).str());
    if (!submap) {
      return false;
    }
    submap->id += start_from_frame_id;
    submap->session_id = session_id;

    // Adaptively determine the voxel resolution based on the median distance
    const int max_scan_count = 256;
    const double dist_median = gtsam_points::median_distance(submap->frame, max_scan_count);
    const double p =
      std::max(0.0, std::min(1.0, (dist_median - params.submap_voxel_resolution_dmin) / (params.submap_voxel_resolution_dmax - params.submap_voxel_resolution_dmin)));
    const double base_resolution = params.submap_voxel_resolution + p * (params.submap_voxel_resolution_max - params.submap_voxel_resolution);

    gtsam_points::PointCloud::Ptr subsampled_submap;
    if (params.randomsampling_rate > 0.99) {
      subsampled_submap = submap->frame;
    } else {
      subsampled_submap = gtsam_points::random_sampling(submap->frame, params.randomsampling_rate, mt);
    }

    submaps.push_back(submap);
    submaps.back()->voxelmaps.clear();
    subsampled_submaps.push_back(subsampled_submap);
    occ_grids_.push_back(create_occupancy_grid(*subsampled_submap, base_resolution));

    if (params.enable_gpu) {
#ifdef GTSAM_POINTS_USE_CUDA
      subsampled_submaps.back() = gtsam_points::PointCloudGPU::clone(*subsampled_submaps.back());

      for (int j = 0; j < params.submap_voxelmap_levels; j++) {
        const double resolution = base_resolution * std::pow(params.submap_voxelmap_scaling_factor, j);
        auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(resolution);
        voxelmap->insert(*subsampled_submaps.back());
        submaps.back()->voxelmaps.push_back(voxelmap);
      }
#else
      logger->warn("GPU is enabled for global_mapping but gtsam_points was built without CUDA!!");
#endif
    } else {
      for (int j = 0; j < params.submap_voxelmap_levels; j++) {
        const double resolution = base_resolution * std::pow(params.submap_voxelmap_scaling_factor, j);
        auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
        voxelmap->insert(*subsampled_submaps.back());
        submaps.back()->voxelmaps.push_back(voxelmap);
      }
    }

    Callbacks::on_insert_submap(submap);
  }

  gtsam::Values values, loaded_values;
  gtsam::NonlinearFactorGraph graph, loaded_graph;
  bool needs_recover = false;

  logger->info("deserializing factor graph");
  try {
    gtsam::deserializeFromBinaryFile(path + "/graph.bin", loaded_graph);
  } catch (boost::archive::archive_exception e) {
    logger->error("failed to deserialize factor graph!!");
    logger->error(e.what());
  } catch (std::exception& e) {
    logger->error("failed to deserialize factor graph!!");
    logger->error(e.what());
    needs_recover = true;
  }

  logger->info("deserializing values");
  try {
    gtsam::deserializeFromBinaryFile(path + "/values.bin", loaded_values);
  } catch (boost::archive::archive_exception e) {
    logger->error("failed to deserialize values!!");
    logger->error(e.what());
  } catch (std::exception& e) {
    logger->error("failed to deserialize values!!");
    logger->error(e.what());
    needs_recover = true;
  }

  // remap keys in graph and values if dump previously loaded
  if (start_from_frame_id > 0) {
    std::map<gtsam::Key, gtsam::Key> rekey_mapping;
    for (int i = 0; i < num_submaps; i++) {
      rekey_mapping[X(i)] = X(i + start_from_frame_id);
      rekey_mapping[E(i * 2)] = E((i + start_from_frame_id) * 2);
      rekey_mapping[E(i * 2 + 1)] = E((i + start_from_frame_id) * 2 + 1);
      rekey_mapping[B(i * 2)] = B((i + start_from_frame_id) * 2);
      rekey_mapping[B(i * 2 + 1)] = B((i + start_from_frame_id) * 2 + 1);
      rekey_mapping[V(i * 2)] = V((i + start_from_frame_id) * 2);
      rekey_mapping[V(i * 2 + 1)] = V((i + start_from_frame_id) * 2 + 1);
    }

    logger->info("removing translation prior factors");
    auto remove_loc = std::remove_if(loaded_graph.begin(), loaded_graph.end(), [](const auto& factor) {
      return dynamic_cast<gtsam::PoseTranslationPrior<gtsam::Pose3>*>(factor.get()) != nullptr;
    });
    logger->info("removed {} prior factors", std::distance(remove_loc, loaded_graph.end()));
    loaded_graph.erase(remove_loc, loaded_graph.end());

    logger->info("removing damping factors");
    remove_loc =
      std::remove_if(loaded_graph.begin(), loaded_graph.end(), [](const auto& factor) { return dynamic_cast<gtsam_points::LinearDampingFactor*>(factor.get()) != nullptr; });
    logger->info("removed {} prior factors", std::distance(remove_loc, loaded_graph.end()));
    loaded_graph.erase(remove_loc, loaded_graph.end());

    logger->info("removing prior factors");
    remove_loc =
      std::remove_if(loaded_graph.begin(), loaded_graph.end(), [](const auto& factor) { return dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(factor.get()) != nullptr; });
    logger->info("removed {} prior factors", std::distance(remove_loc, loaded_graph.end()));
    loaded_graph.erase(remove_loc, loaded_graph.end());

    // rekey graph
    graph = loaded_graph;
    logger->info("rekeying factors");
    graph = graph.rekey(rekey_mapping);

    // rekey values
    for (auto it = loaded_values.begin(); it != loaded_values.end(); ++it) {
      auto matched_key = rekey_mapping.find(it->key);
      if (matched_key != rekey_mapping.end()) {
        values.insert(matched_key->second, it->value);
      } else {
        logger->warn("No remapping found for Value with key {}, keeping it as is", gtsam::Symbol(it->key).string());
        values.insert(it->key, it->value);
      }
    }
  } else {
    graph = loaded_graph;
    values = loaded_values;
  }

  logger->info("creating matching cost factors");
  for (const auto& factor : matching_cost_factors) {
    const auto type = std::get<0>(factor);
    const auto first = std::get<1>(factor) + start_from_frame_id;
    const auto second = std::get<2>(factor) + start_from_frame_id;

    if (type == "vgicp" || type == "vgicp_gpu") {
      if (params.enable_gpu) {
#ifdef GTSAM_POINTS_USE_CUDA
        const auto stream_buffer = std::any_cast<std::shared_ptr<gtsam_points::StreamTempBufferRoundRobin>>(stream_buffer_roundrobin)->get_stream_buffer();
        const auto& stream = stream_buffer.first;
        const auto& buffer = stream_buffer.second;

        for (const auto& voxelmap : submaps[first]->voxelmaps) {
          graph.emplace_shared<gtsam_points::IntegratedVGICPFactorGPU>(X(first), X(second), voxelmap, subsampled_submaps[second], stream, buffer);
        }
#else
        logger->warn("GPU is enabled but gtsam_points was built without CUDA!!");
#endif
      } else {
        for (const auto& voxelmap : submaps[first]->voxelmaps) {
          graph.emplace_shared<gtsam_points::IntegratedVGICPFactor>(X(first), X(second), voxelmap, subsampled_submaps[second]);
        }
      }
    } else if (type == "epcd_gicp") {
      graph.add(create_epcd_factor(first, second));
    } else {
      logger->warn("unsupported matching cost factor type ({})", type);
    }
  }

  const size_t num_factors_before = graph.size();
  const auto remove_loc = std::remove_if(graph.begin(), graph.end(), [](const auto& factor) { return factor == nullptr; });
  graph.erase(remove_loc, graph.end());
  if (graph.size() != num_factors_before) {
    logger->warn("removed {} invalid factors", num_factors_before - graph.size());
    needs_recover = true;
  }

  if (needs_recover) {
    logger->warn("recovering factor graph");
    const auto recovered = recover_graph(graph, values, start_from_frame_id);
    logger->warn("add {} factors and {} values", recovered.first.size(), recovered.second.size());

    graph.add(recovered.first);
    values.insert_or_assign(recovered.second);
  }

  if (start_from_frame_id <= 0) {
    logger->info("optimize");
    Callbacks::on_smoother_update(*isam2, graph, values);
    auto result = update_isam2(graph, values);
    Callbacks::on_smoother_update_result(*isam2, result);

    update_submaps();
    Callbacks::on_update_submaps(submaps);
  } else {
    logger->info("skip optimization");
    this->new_factors->add(graph);
    this->new_values->insert(values);
  }

  logger->info("done");
  session_id++;

  return true;
}

void GlobalMapping::recover_graph() {
  const auto recovered = recover_graph(isam2->getFactorsUnsafe(), isam2->calculateEstimate(), 0);
  update_isam2(recovered.first, recovered.second);
}

// Recover the graph by adding missing values and factors
std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> GlobalMapping::recover_graph(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values, int start_from_frame_id)
  const {
  logger->info("recovering graph");
  bool enable_imu = false;
  for (const auto& value : values) {
    const char chr = gtsam::Symbol(value.key).chr();
    enable_imu |= (chr == 'e' || chr == 'v' || chr == 'b');
  }
  for (const auto& factor : graph) {
    enable_imu |= dynamic_cast<gtsam::ImuFactor*>(factor.get()) != nullptr;
  }

  logger->info("enable_imu={}", enable_imu);

  logger->info("creating connectivity map");
  bool prior_exists = false;
  std::unordered_map<gtsam::Key, std::set<gtsam::Key>> connectivity_map;
  for (const auto& factor : graph) {
    if (!factor) {
      continue;
    }

    for (const auto key : factor->keys()) {
      for (const auto key2 : factor->keys()) {
        connectivity_map[key].insert(key2);
      }
    }

    if (factor->keys().size() == 1 && factor->keys()[0] == X(0)) {
      prior_exists |= dynamic_cast<gtsam_points::LinearDampingFactor*>(factor.get()) != nullptr;
    }
  }

  logger->info("fixing missing values and factors");
  const auto prior_noise3 = gtsam::noiseModel::Isotropic::Precision(3, 1e6);
  const auto prior_noise6 = gtsam::noiseModel::Isotropic::Precision(6, 1e6);

  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;

  if (!prior_exists) {
    logger->warn("X0 prior is missing");
    new_factors.emplace_shared<gtsam_points::LinearDampingFactor>(X(0), 6, params.init_pose_damping_scale);
  }

  for (int i = start_from_frame_id; i < submaps.size(); i++) {
    if (!values.exists(X(i))) {
      logger->warn("X{} is missing", i);
      new_values.insert(X(i), gtsam::Pose3(submaps[i]->T_world_origin.matrix()));
    }

    if (connectivity_map[X(i)].count(X(i + 1)) == 0 && i != submaps.size() - 1) {
      logger->warn("X{} -> X{} is missing", i, i + 1);

      const Eigen::Isometry3d delta = submaps[i]->origin_odom_frame()->T_world_sensor().inverse() * submaps[i + 1]->origin_odom_frame()->T_world_sensor();
      new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(i), X(i + 1), gtsam::Pose3(delta.matrix()), prior_noise6);
    }

    if (!enable_imu) {
      continue;
    }

    const auto submap = submaps[i];
    const gtsam::imuBias::ConstantBias imu_biasL(submap->frames.front()->imu_bias);
    const gtsam::imuBias::ConstantBias imu_biasR(submap->frames.back()->imu_bias);
    const Eigen::Vector3d v_origin_imuL = submap->T_world_origin.linear().inverse() * submap->frames.front()->v_world_imu;
    const Eigen::Vector3d v_origin_imuR = submap->T_world_origin.linear().inverse() * submap->frames.back()->v_world_imu;

    if (i != 0) {
      if (!values.exists(E(i * 2))) {
        logger->warn("E{} is missing", i * 2);
        new_values.insert(E(i * 2), gtsam::Pose3((submap->T_world_origin * submap->T_origin_endpoint_L).matrix()));
      }
      if (!values.exists(V(i * 2))) {
        logger->warn("V{} is missing", i * 2);
        new_values.insert(V(i * 2), (submap->T_world_origin.linear() * v_origin_imuL).eval());
      }
      if (!values.exists(B(i * 2))) {
        logger->warn("B{} is missing", i * 2);
        new_values.insert(B(i * 2), imu_biasL);
      }

      if (connectivity_map[X(i)].count(E(i * 2)) == 0) {
        logger->warn("X{} -> E{} is missing", i, i * 2);
        new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(i), E(i * 2), gtsam::Pose3(submap->T_origin_endpoint_L.matrix()), prior_noise6);
      }
      if (connectivity_map[X(i)].count(V(i * 2)) == 0) {
        logger->warn("X{} -> V{} is missing", i, i * 2);
        new_factors.emplace_shared<gtsam_points::RotateVector3Factor>(X(i), V(i * 2), v_origin_imuL, prior_noise3);
      }
      if (connectivity_map[B(i * 2)].count(B(i * 2)) == 0) {
        logger->warn("B{} -> B{} is missing", i * 2, i * 2);
        new_factors.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(i * 2), imu_biasL, prior_noise6);
      }

      if (connectivity_map[B(i * 2)].count(B(i * 2 + 1)) == 0) {
        logger->warn("B{} -> B{} is missing", i * 2, i * 2 + 1);
        new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(i * 2), B(i * 2 + 1), gtsam::imuBias::ConstantBias(), prior_noise6);
      }
    }

    if (!values.exists(E(i * 2 + 1))) {
      logger->warn("E{} is missing", i * 2 + 1);
      new_values.insert(E(i * 2 + 1), gtsam::Pose3((submap->T_world_origin * submap->T_origin_endpoint_R).matrix()));
    }
    if (!values.exists(V(i * 2 + 1))) {
      logger->warn("V{} is missing", i * 2 + 1);
      new_values.insert(V(i * 2 + 1), (submap->T_world_origin.linear() * v_origin_imuR).eval());
    }
    if (!values.exists(B(i * 2 + 1))) {
      logger->warn("B{} is missing", i * 2 + 1);
      new_values.insert(B(i * 2 + 1), imu_biasR);
    }

    if (connectivity_map[X(i)].count(E(i * 2 + 1)) == 0) {
      logger->warn("X{} -> E{} is missing", i, i * 2 + 1);
      new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(i), E(i * 2 + 1), gtsam::Pose3(submap->T_origin_endpoint_R.matrix()), prior_noise6);
    }
    if (connectivity_map[X(i)].count(V(i * 2 + 1)) == 0) {
      logger->warn("X{} -> V{} is missing", i, i * 2 + 1);
      new_factors.emplace_shared<gtsam_points::RotateVector3Factor>(X(i), V(i * 2 + 1), v_origin_imuR, prior_noise3);
    }
    if (connectivity_map[B(i * 2 + 1)].count(B(i * 2 + 1)) == 0) {
      logger->warn("B{} -> B{} is missing", i * 2 + 1, i * 2 + 1);
      new_factors.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(i * 2 + 1), imu_biasR, prior_noise6);
    }
  }

  logger->info("recovering done");

  return {new_factors, new_values};
}

}  // namespace glim
