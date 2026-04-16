#pragma once

#include <any>
#include <deque>
#include <random>
#include <memory>
#include <glim/mapping/sub_mapping_base.hpp>

namespace gtsam {
class Values;
class NonlinearFactorGraph;
class PreintegratedImuMeasurements;
}  // namespace gtsam

namespace glim {

class IMUIntegration;
class CloudDeskewing;
class CloudCovarianceEstimation;

/**
 * @brief Sub mapping parameters
 */
struct SubMappingParams {
public:
  SubMappingParams();
  ~SubMappingParams();

public:
  bool enable_gpu;
  bool enable_imu;
  bool enable_optimization;
  // Keyframe update strategy params
  int max_num_keyframes;
  std::string keyframe_update_strategy;
  int keyframe_update_min_points;
  double keyframe_update_interval_rot;
  double keyframe_update_interval_trans;
  double max_keyframe_overlap;

  bool create_between_factors;
  std::string between_registration_type;

  std::string registration_error_factor_type;
  double keyframe_randomsampling_rate;
  double keyframe_voxel_resolution;
  int keyframe_voxelmap_levels;
  double keyframe_voxelmap_scaling_factor;
  double epcd_max_correspondence_distance = 1.0;
  bool epcd_enable_exact_downsampling = true;
  int epcd_num_threads = 4;
  int epcd_coreset_target_num_points = 96;
  int epcd_coreset_num_clusters = 64;
  double epcd_deferred_sampling_translation = 0.25;
  double epcd_deferred_sampling_rotation = 0.004363323129985824;
  double epcd_rebuild_translation = 1.0;
  double epcd_rebuild_rotation = 0.017453292519943295;

  double submap_downsample_resolution;
  double submap_voxel_resolution;
  int submap_target_num_points;
};

/**
 * @brief Sub mapping
 */
class SubMapping : public SubMappingBase {
public:
  SubMapping(const SubMappingParams& params = SubMappingParams());
  virtual ~SubMapping() override;

  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual void insert_frame(const EstimationFrame::ConstPtr& odom_frame) override;

  virtual std::vector<SubMap::Ptr> get_submaps() override;

  virtual std::vector<SubMap::Ptr> submit_end_of_sequence() override;

private:
  void insert_keyframe(const int current, const EstimationFrame::ConstPtr& odom_frame);

  SubMap::Ptr create_submap(bool force_create = false) const;

private:
  using Params = SubMappingParams;
  Params params;

  std::mt19937 mt;
  int submap_count;

  std::unique_ptr<IMUIntegration> imu_integration;
  std::unique_ptr<CloudDeskewing> deskewing;
  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  std::shared_ptr<void> stream;
  std::shared_ptr<void> stream_buffer_roundrobin;

  std::deque<EstimationFrame::ConstPtr> delayed_input_queue;
  std::vector<EstimationFrame::ConstPtr> odom_frames;

  std::vector<int> keyframe_indices;
  std::vector<EstimationFrame::Ptr> keyframes;

  std::unique_ptr<gtsam::Values> values;
  std::unique_ptr<gtsam::NonlinearFactorGraph> graph;

  std::vector<SubMap::Ptr> submap_queue;

  std::shared_ptr<void> tbb_task_arena;
};

}  // namespace glim
