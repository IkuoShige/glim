#pragma once

#include <glim/odometry/odometry_estimation_imu.hpp>

namespace gtsam_points {

class GaussianVoxelMapCPU;
struct FlatContainer;
template <typename VoxelContents>
class IncrementalVoxelMap;
using iVox = IncrementalVoxelMap<FlatContainer>;
}  // namespace gtsam_points

namespace glim {

/**
 * @brief Parameters for OdometryEstimationCPU
 */
struct OdometryEstimationCPUParams : public OdometryEstimationIMUParams {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationCPUParams();
  virtual ~OdometryEstimationCPUParams();

  enum class KeyframeUpdateStrategy { OVERLAP, DISPLACEMENT };

public:
  // Registration params
  std::string registration_type;    ///< Registration type (GICP or VGICP)
  int max_iterations;               ///< Maximum number of iterations
  int lru_thresh;                   ///< LRU cache threshold
  double target_downsampling_rate;  ///< Downsampling rate for points to be inserted into the target

  double ivox_resolution;  ///< iVox resolution (for GICP)
  double ivox_min_dist;    ///< Minimum distance between points in an iVox cell (for GICP)

  double vgicp_resolution;               ///< Voxelmap resolution (for VGICP)
  int vgicp_voxelmap_levels;             ///< Multi-resolution voxelmap levesl (for VGICP)
  double vgicp_voxelmap_scaling_factor;  ///< Multi-resolution voxelmap scaling factor (for VGICP)

  double epcd_voxel_resolution;               ///< Voxelmap resolution for CPU keyframe overlap
  double epcd_voxel_resolution_max;           ///< Maximum adaptive voxelmap resolution for CPU keyframe overlap
  double epcd_voxel_resolution_dmin;          ///< Minimum median distance for adaptive voxelmap resolution
  double epcd_voxel_resolution_dmax;          ///< Maximum median distance for adaptive voxelmap resolution
  int epcd_voxelmap_levels;                   ///< Multi-resolution voxelmap levels for CPU keyframe overlap
  double epcd_voxelmap_scaling_factor;        ///< Multi-resolution voxelmap scaling factor for CPU keyframe overlap
  int epcd_max_num_keyframes;                 ///< Maximum number of active EPCD keyframes
  int epcd_full_connection_window_size;       ///< Full frame-to-frame connection window size
  KeyframeUpdateStrategy epcd_keyframe_strategy;
  double epcd_keyframe_min_overlap;
  double epcd_keyframe_max_overlap;
  double epcd_keyframe_delta_trans;
  double epcd_keyframe_delta_rot;
  double epcd_max_correspondence_distance;
  bool epcd_enable_exact_downsampling;
  int epcd_coreset_target_num_points;
  int epcd_coreset_num_clusters;
  double epcd_deferred_sampling_translation;
  double epcd_deferred_sampling_rotation;
  double epcd_rebuild_translation;
  double epcd_rebuild_rotation;
};

/**
 * @brief CPU-based semi-tightly coupled LiDAR-IMU odometry
 */
class OdometryEstimationCPU : public OdometryEstimationIMU {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationCPU(const OdometryEstimationCPUParams& params = OdometryEstimationCPUParams());
  virtual ~OdometryEstimationCPU() override;

private:
  virtual void create_frame(EstimationFrame::Ptr& frame) override;
  virtual gtsam::NonlinearFactorGraph create_factors(const int current, const gtsam_points::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) override;
  virtual void update_frames(const int current, const gtsam::NonlinearFactorGraph& new_factors) override;

  virtual void fallback_smoother() override;

  void update_target(const int current, const Eigen::Isometry3d& T_target_imu);
  void update_keyframes_overlap(int current);
  void update_keyframes_displacement(int current);

private:
  // Registration params
  std::mt19937 mt;                                                                   ///< RNG
  Eigen::Isometry3d last_T_target_imu;                                               ///< Last IMU pose w.r.t. target model
  std::vector<std::shared_ptr<gtsam_points::GaussianVoxelMapCPU>> target_voxelmaps;  ///< VGICP target voxelmap
  std::shared_ptr<gtsam_points::iVox> target_ivox;                                   ///< GICP target iVox
  EstimationFrame::ConstPtr target_ivox_frame;                                       ///< Target points (just for visualization)
  std::vector<EstimationFrame::ConstPtr> keyframes;                                  ///< EPCD keyframes
};

}  // namespace glim
