/*
 * MIT License
 * Copyright (c) 2025 Senming Tan
 */

#include "fsgp_bgk_cpp/global_map_manager.hpp"

#include <cmath>
#include <algorithm>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace fsgp_bgk
{

GlobalMapManager::GlobalMapManager(const GlobalMapConfig & config)
: config_(config)
{
  init_grids();
}

void GlobalMapManager::init_grids()
{
  grid_size_ = static_cast<int>(config_.max_radius * 2.0 / config_.grid_resolution);
  grid_half_ = static_cast<int>(config_.max_radius / config_.grid_resolution);

  const size_t total_cells = static_cast<size_t>(grid_size_ * grid_size_);
  global_grid_.assign(total_cells, 0.0f);
  global_grid_ldd_.assign(total_cells, 0.0f);

  const float init_log_odds = std::log(
    static_cast<float>(config_.occupancy_threshold) /
    (1.0f - static_cast<float>(config_.occupancy_threshold)));
  log_odds_grid_.assign(total_cells, init_log_odds);

  // High-resolution output grid
  high_res_half_ = static_cast<int>(config_.max_radius / config_.publish_resolution);
  const int high_res_size = high_res_half_ * 2;

  high_res_x_.resize(high_res_size);
  high_res_y_.resize(high_res_size);

  for (int i = 0; i < high_res_size; ++i) {
    high_res_x_[i] = (i - high_res_half_) * static_cast<float>(config_.publish_resolution);
    high_res_y_[i] = (i - high_res_half_) * static_cast<float>(config_.publish_resolution);
  }
}

float GlobalMapManager::observation_model(float traversability) const
{
  if (config_.binarization_condition) {
    const float threshold = 1.0f - static_cast<float>(config_.obstacle_threshold);
    if (traversability > threshold) {
      return std::log(0.95f / 0.05f);
    } else {
      return std::log(0.05f / 0.95f);
    }
  } else {
    traversability = std::clamp(traversability, 0.001f, 0.999f);
    return std::log(traversability / (1.0f - traversability));
  }
}

void GlobalMapManager::transform_to_global(
  float local_x, float local_y, float local_z,
  const std::array<double, 6> & pose,
  float & global_x, float & global_y, float & global_z) const
{
  const double yaw = pose[5];
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  global_x = static_cast<float>(local_x * cos_yaw - local_y * sin_yaw + pose[0]);
  global_y = static_cast<float>(local_x * sin_yaw + local_y * cos_yaw + pose[1]);
  global_z = static_cast<float>(local_z + pose[2]);
}

void GlobalMapManager::smooth_grid(
  std::vector<float> & grid, int width, int height, int kernel_size) const
{
  if (kernel_size <= 1) return;

  const int half_k = kernel_size / 2;
  std::vector<float> temp(grid.size(), 0.0f);

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      float sum = 0.0f;
      int count = 0;

      for (int ky = -half_k; ky <= half_k; ++ky) {
        for (int kx = -half_k; kx <= half_k; ++kx) {
          const int nx = x + kx;
          const int ny = y + ky;
          if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
            sum += grid[ny * width + nx];
            ++count;
          }
        }
      }

      temp[y * width + x] = (count > 0) ? sum / count : 0.0f;
    }
  }

  grid = std::move(temp);
}

float GlobalMapManager::interpolate(
  const std::vector<float> & grid,
  const std::vector<float> & grid_x,
  const std::vector<float> & grid_y,
  float query_x, float query_y) const
{
  if (grid_x.empty() || grid_y.empty()) return 1.0f;

  // Find indices in the low-res grid
  const float fx = (query_x - grid_x.front()) / (grid_x.back() - grid_x.front()) * (grid_x.size() - 1);
  const float fy = (query_y - grid_y.front()) / (grid_y.back() - grid_y.front()) * (grid_y.size() - 1);

  const int ix = static_cast<int>(std::floor(fx));
  const int iy = static_cast<int>(std::floor(fy));

  const int width = static_cast<int>(grid_x.size());
  const int height = static_cast<int>(grid_y.size());

  if (ix < 0 || ix >= width - 1 || iy < 0 || iy >= height - 1) {
    return 1.0f;  // fill value for out-of-bounds
  }

  const float dx = fx - ix;
  const float dy = fy - iy;

  // Bilinear interpolation
  const float v00 = grid[iy * width + ix];
  const float v10 = grid[iy * width + ix + 1];
  const float v01 = grid[(iy + 1) * width + ix];
  const float v11 = grid[(iy + 1) * width + ix + 1];

  return v00 * (1 - dx) * (1 - dy) +
         v10 * dx * (1 - dy) +
         v01 * (1 - dx) * dy +
         v11 * dx * dy;
}

void GlobalMapManager::update(
  const std::array<double, 6> & pose,
  const TraversabilityResult & result)
{
  const size_t n = result.traversability.size();
  if (n == 0) return;

  const float pose_x = static_cast<float>(pose[0]);
  const float pose_y = static_cast<float>(pose[1]);

  for (size_t i = 0; i < n; ++i) {
    // Transform local grid point to global
    float gx, gy, gz;
    transform_to_global(
      result.grid_x[i], result.grid_y[i], result.mean_heights[i],
      pose, gx, gy, gz);

    // Map to grid indices
    const int ix = static_cast<int>((gx - pose_x) / config_.grid_resolution) + grid_half_;
    const int iy = static_cast<int>((gy - pose_y) / config_.grid_resolution) + grid_half_;

    if (ix < 0 || ix >= grid_size_ || iy < 0 || iy >= grid_size_) {
      continue;
    }

    const size_t idx = static_cast<size_t>(iy * grid_size_ + ix);
    const float trav = 1.0f - result.traversability[i];  // convert to intensity

    // Update log-odds
    log_odds_grid_[idx] += observation_model(trav);
    log_odds_grid_[idx] = std::clamp(log_odds_grid_[idx], -10.0f, 10.0f);

    // Update grids
    global_grid_ldd_[idx] = 1.0f / (1.0f + std::exp(-log_odds_grid_[idx]));
    global_grid_[idx] = trav;
  }

  // Smooth both grids
  smooth_grid(global_grid_, grid_size_, grid_size_, config_.smooth_kernel_size);
  smooth_grid(global_grid_ldd_, grid_size_, grid_size_, config_.smooth_kernel_size);
}

void GlobalMapManager::generate_output_clouds(
  const std::array<double, 6> & pose,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  sensor_msgs::msg::PointCloud2 & raw_cloud,
  sensor_msgs::msg::PointCloud2 & ldd_cloud) const
{
  const int high_res_size = high_res_half_ * 2;
  const size_t total_points = static_cast<size_t>(high_res_size * high_res_size);

  // Build low-res grid coordinate arrays for interpolation
  std::vector<float> low_res_x(grid_size_);
  std::vector<float> low_res_y(grid_size_);
  for (int i = 0; i < grid_size_; ++i) {
    low_res_x[i] = (i - grid_half_) * static_cast<float>(config_.grid_resolution)
                    + static_cast<float>(pose[0]);
    low_res_y[i] = (i - grid_half_) * static_cast<float>(config_.grid_resolution)
                    + static_cast<float>(pose[1]);
  }

  // Prepare point cloud data
  std::vector<float> points_x(total_points);
  std::vector<float> points_y(total_points);
  std::vector<float> points_z(total_points);
  std::vector<float> raw_intensity(total_points);
  std::vector<float> ldd_intensity(total_points);

  const float base_z = static_cast<float>(pose[2]) - 0.15f;
  const float max_r = static_cast<float>(config_.max_radius)
                      - static_cast<float>(config_.publish_resolution) * 2.0f;

  for (int ix = 0; ix < high_res_size; ++ix) {
    for (int iy = 0; iy < high_res_size; ++iy) {
      const size_t idx = static_cast<size_t>(ix * high_res_size + iy);

      const float lx = high_res_x_[ix];
      const float ly = high_res_y_[iy];

      points_x[idx] = lx + static_cast<float>(pose[0]);
      points_y[idx] = ly + static_cast<float>(pose[1]);
      points_z[idx] = base_z;

      // Check if within valid radius
      const float r = std::hypot(lx, ly);
      if (r > max_r) {
        raw_intensity[idx] = 1.0f;
        ldd_intensity[idx] = 1.0f;
      } else {
        raw_intensity[idx] = interpolate(
          global_grid_, low_res_x, low_res_y, points_x[idx], points_y[idx]);
        ldd_intensity[idx] = interpolate(
          global_grid_ldd_, low_res_x, low_res_y, points_x[idx], points_y[idx]);
      }
    }
  }

  // Helper to build a PointCloud2 message with XYZI fields
  auto build_cloud = [&](
    sensor_msgs::msg::PointCloud2 & cloud,
    const std::vector<float> & intensity)
  {
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(2, "xyz", "intensity");
    modifier.resize(total_points);

    cloud.header.frame_id = frame_id;
    cloud.header.stamp = stamp;

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(cloud, "intensity");

    for (size_t i = 0; i < total_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
      *iter_x = points_x[i];
      *iter_y = points_y[i];
      *iter_z = points_z[i];
      *iter_i = intensity[i];
    }
  };

  build_cloud(raw_cloud, raw_intensity);
  build_cloud(ldd_cloud, ldd_intensity);
}

void GlobalMapManager::reset()
{
  init_grids();
}

void GlobalMapManager::update_config(const GlobalMapConfig & config)
{
  config_ = config;
  init_grids();
}

}  // namespace fsgp_bgk
