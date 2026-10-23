#pragma once

#include <any>
#include <memory>
#include <random>
#include <unordered_map>
#include <vector>
#include <Eigen/Geometry>
#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Factor.h>
#include <glim/factors/exact_gicp_factor.hpp>
#include <glim/mapping/global_mapping_base.hpp>

namespace gtsam_points {
struct NearestNeighborSearch;
class FastOccupancyGrid;
}  // namespace gtsam_points

namespace gtsam {
class Values;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_points {
class ISAM2Ext;
class StreamTempBufferRoundRobin;
struct ISAM2ResultExt;
}  // namespace gtsam_points

namespace glim {

class IMUIntegration;

/**
 * @brief Global mapping parameters
 */
struct GlobalMappingParams {
public:
  GlobalMappingParams();
  ~GlobalMappingParams();

public:
  bool enable_gpu;
  bool enable_imu;
  bool enable_optimization;
  bool enable_between_factors;
  std::string between_registration_type;

  std::string registration_error_factor_type;
  double submap_voxel_resolution;
  double submap_voxel_resolution_max;
  double submap_voxel_resolution_dmin;
  double submap_voxel_resolution_dmax;
  int submap_voxelmap_levels;
  double submap_voxelmap_scaling_factor;
  ExactGICPFactorParams epcd;

  double randomsampling_rate;
  double max_implicit_loop_distance;
  double min_implicit_loop_overlap;
  bool enable_loop_verification;  // register non-recent overlapping pairs coarse-to-fine before adding registration error factors
  int loop_coarse_levels;         // coarse-to-fine stages at max_correspondence_distance * 2^k (k = levels-1, ..., 0)
  int loop_consistency_submaps;   // number of subsequent submaps that must agree with a loop correction before it is inserted

  bool use_isam2_dogleg;
  double isam2_relinearize_skip;
  double isam2_relinearize_thresh;

  double init_pose_damping_scale;
};

/**
 * @brief Global mapping
 */
class GlobalMapping : public GlobalMappingBase {
public:
  GlobalMapping(const GlobalMappingParams& params = GlobalMappingParams());
  virtual ~GlobalMapping();

  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual void insert_submap(const SubMap::Ptr& submap) override;

  virtual void find_overlapping_submaps(double min_overlap) override;
  virtual void optimize() override;

  virtual void save(const std::string& path) override;
  virtual gtsam_points::PointCloud::Ptr export_points() override;

  /**
   * @brief Load a mapping result from a dumped directory
   * @param path Input dump path
   */
  bool load(const std::string& path);

private:
  void insert_submap(int current, const SubMap::Ptr& submap);

  std::shared_ptr<gtsam::NonlinearFactorGraph> create_between_factors(int current) const;
  std::shared_ptr<gtsam::NonlinearFactorGraph> create_matching_cost_factors(int current);
  std::shared_ptr<gtsam::NonlinearFactorGraph> create_registration_factors(int target, int source) const;
  std::shared_ptr<gtsam_points::FastOccupancyGrid> create_occupancy_grid(const gtsam_points::PointCloud& points, double base_resolution) const;
  gtsam::NonlinearFactor::shared_ptr create_epcd_factor(int target, int source) const;

  struct LoopAlignment {
    bool valid = false;
    Eigen::Isometry3d T_target_source = Eigen::Isometry3d::Identity();
    gtsam::Matrix6 H = gtsam::Matrix6::Zero();
  };

  // A registration result between a group of past submaps (one revisit) and a new submap, awaiting confirmation
  struct LoopHypothesis {
    std::vector<int> targets;
    int representative = -1;
    int source = -1;
    Eigen::Isometry3d T_world_source = Eigen::Isometry3d::Identity();  // source pose implied by the registration
    LoopAlignment alignment;
    bool in_basin = false;  // the current estimate already agrees with the registration
    int confirmations = 0;
    bool rejected = false;
  };

  // A BetweenFactor that pulls an out-of-basin revisit in, kept until registration error factors can replace it
  struct LoopCorrection {
    int target;
    int source;
    gtsam::NonlinearFactor::shared_ptr factor;
    gtsam::FactorIndex isam2_index = 0;  // valid once the factor has been stored in isam2
  };

  std::shared_ptr<gtsam::NonlinearFactorGraph> create_verified_loop_factors(int current, const std::vector<std::pair<int, double>>& candidates);
  std::shared_ptr<gtsam::NonlinearFactorGraph> confirm_loop(const LoopHypothesis& hypothesis);
  LoopAlignment align_coarse_to_fine(int target, int source, const Eigen::Isometry3d& init_T_target_source) const;
  gtsam_points::PointCloud::ConstPtr coarse_frame(int submap_id, double resolution) const;
  double source_displacement(int source, const Eigen::Isometry3d& T_a, const Eigen::Isometry3d& T_b) const;

  void update_submaps();
  gtsam_points::ISAM2ResultExt update_isam2(const gtsam::NonlinearFactorGraph& new_factors, const gtsam::Values& new_values, const gtsam::FactorIndices& remove_indices = {});

  void recover_graph() override;
  std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> recover_graph(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values, int start_from_frame_id) const;

private:
  using Params = GlobalMappingParams;
  Params params;

  std::mt19937 mt;
  int session_id;

  std::unique_ptr<IMUIntegration> imu_integration;
  std::any stream_buffer_roundrobin;

  std::vector<SubMap::Ptr> submaps;
  std::vector<gtsam_points::PointCloud::ConstPtr> subsampled_submaps;
  mutable std::unordered_map<int, std::shared_ptr<const gtsam_points::NearestNeighborSearch>> kdtree_cache_;
  std::vector<std::shared_ptr<gtsam_points::FastOccupancyGrid>> occ_grids_;

  std::vector<LoopHypothesis> pending_loops_;
  std::vector<LoopCorrection> new_loop_corrections_;  // added to new_factors in this update
  std::vector<LoopCorrection> loop_corrections_;      // stored in isam2
  gtsam::FactorIndices factors_to_remove_;            // loop corrections replaced by registration error factors

  std::unique_ptr<gtsam::Values> new_values;
  std::unique_ptr<gtsam::NonlinearFactorGraph> new_factors;

  std::unique_ptr<gtsam_points::ISAM2Ext> isam2;

  std::shared_ptr<void> tbb_task_arena;
};
}  // namespace glim
