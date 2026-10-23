#pragma once

#include <any>
#include <memory>
#include <random>
#include <unordered_map>
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
  double epcd_max_correspondence_distance = 1.0;
  bool epcd_enable_exact_downsampling = true;
  int epcd_num_threads = 4;
  int epcd_coreset_target_num_points = 96;
  int epcd_coreset_num_clusters = 64;
  double epcd_deferred_sampling_translation = 0.25;
  double epcd_deferred_sampling_rotation = 0.004363323129985824;
  double epcd_rebuild_translation = 1.0;
  double epcd_rebuild_rotation = 0.017453292519943295;

  double randomsampling_rate;
  double max_implicit_loop_distance;
  double min_implicit_loop_overlap;
  int matching_cost_factor_window;  // submap index distance threshold; pairs beyond this use BetweenFactor (-1 = unlimited)

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
  std::shared_ptr<gtsam::NonlinearFactorGraph> create_matching_cost_factors(int current) const;
  std::shared_ptr<gtsam::NonlinearFactorGraph> align_and_create_between_factor(int i, int j) const;

  void update_submaps();
  gtsam_points::ISAM2ResultExt update_isam2(const gtsam::NonlinearFactorGraph& new_factors, const gtsam::Values& new_values);

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

  std::unique_ptr<gtsam::Values> new_values;
  std::unique_ptr<gtsam::NonlinearFactorGraph> new_factors;

  std::unique_ptr<gtsam_points::ISAM2Ext> isam2;

  std::shared_ptr<void> tbb_task_arena;
};
}  // namespace glim
