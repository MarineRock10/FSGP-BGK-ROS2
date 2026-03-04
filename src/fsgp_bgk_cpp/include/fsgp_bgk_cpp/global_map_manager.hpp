/*
 * MIT License
 * Copyright (c) 2025 Senming Tan
 *
 * Global map management for FSGP-BGK traversability system.
 * Maintains a log-odds occupancy grid, performs smoothing,
 * and generates high-resolution output point clouds.
 */

#ifndef FSGP_BGK_CPP__GLOBAL_MAP_MANAGER_HPP_
#define FSGP_BGK_CPP__GLOBAL_MAP_MANAGER_HPP_

#include <vector>
#include <array>
#include <Eigen/Dense>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace fsgp_bgk
{

struct GlobalMapConfig
{
  double grid_resolution = 0.2;
  double publish_resolution = 0.1;
  double max_radius = 2.5;
  double occupancy_threshold = 0.5;
  double obstacle_threshold = 0.5;
  int smooth_kernel_size = 3;
  bool binarization_condition = true;
};

/// A single traversability observation from the Python algorithm
struct TraversabilityResult
{
  std::vector<float> grid_x;
  std::vector<float> grid_y;
  std::vector<float> mean_heights;
  std::vector<float> traversability;
};

class GlobalMapManager
{
public:
  explicit GlobalMapManager(const GlobalMapConfig & config);

  /// Update the global grid with new local traversability observations
  /// pose: [x, y, z, roll, pitch, yaw]
  void update(
    const std::array<double, 6> & pose,
    const TraversabilityResult & result);

  /// Generate PointCloud2 messages for publishing
  /// Returns raw traversability cloud and log-odds filtered cloud
  void generate_output_clouds(
    const std::array<double, 6> & pose,
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp,
    sensor_msgs::msg::PointCloud2 & raw_cloud,
    sensor_msgs::msg::PointCloud2 & ldd_cloud) const;

  /// Reset the global map
  void reset();

  /// Update configuration at runtime
  void update_config(const GlobalMapConfig & config);

private:
  GlobalMapConfig config_;

  int grid_size_;
  int grid_half_;
  std::vector<float> global_grid_;
  std::vector<float> global_grid_ldd_;
  std::vector<float> log_odds_grid_;

  // High-resolution output grid
  int high_res_half_;
  std::vector<float> high_res_x_;
  std::vector<float> high_res_y_;

  void init_grids();

  /// Compute log-odds observation value
  float observation_model(float traversability) const;

  /// Apply uniform smoothing filter to a grid
  void smooth_grid(std::vector<float> & grid, int width, int height, int kernel_size) const;

  /// Bilinear interpolation on low-res grid to produce high-res output
  float interpolate(
    const std::vector<float> & grid,
    const std::vector<float> & grid_x,
    const std::vector<float> & grid_y,
    float query_x, float query_y) const;

  /// Transform local coordinates to global using pose
  void transform_to_global(
    float local_x, float local_y, float local_z,
    const std::array<double, 6> & pose,
    float & global_x, float & global_y, float & global_z) const;
};

}  // namespace fsgp_bgk

#endif  // FSGP_BGK_CPP__GLOBAL_MAP_MANAGER_HPP_
