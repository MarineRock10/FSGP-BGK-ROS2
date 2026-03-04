/*
 * MIT License
 * Copyright (c) 2025 Senming Tan
 *
 * Point cloud preprocessing for FSGP-BGK traversability system.
 * Handles voxel downsampling, height filtering, radius filtering,
 * and conversion between ROS and PCL formats.
 */

#ifndef FSGP_BGK_CPP__POINT_CLOUD_PREPROCESSOR_HPP_
#define FSGP_BGK_CPP__POINT_CLOUD_PREPROCESSOR_HPP_

#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace fsgp_bgk
{

struct PreprocessorConfig
{
  double voxel_size = 0.3;
  double max_height = 0.5;
  double max_radius = 2.5;
  int min_points_threshold = 10;
};

class PointCloudPreprocessor
{
public:
  explicit PointCloudPreprocessor(const PreprocessorConfig & config);

  /// Convert PointCloud2 message to PCL point cloud, applying height and radius filters
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  filter_cloud(const sensor_msgs::msg::PointCloud2 & msg) const;

  /// Voxel downsample a point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  voxel_downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) const;

  /// Full preprocessing pipeline: parse -> filter -> downsample
  /// Returns flattened float vector [x0,y0,z0, x1,y1,z1, ...] for service call
  std::vector<float>
  preprocess(const sensor_msgs::msg::PointCloud2 & msg, uint32_t & num_points) const;

  /// Update configuration at runtime
  void update_config(const PreprocessorConfig & config);

  const PreprocessorConfig & config() const { return config_; }

private:
  PreprocessorConfig config_;
};

}  // namespace fsgp_bgk

#endif  // FSGP_BGK_CPP__POINT_CLOUD_PREPROCESSOR_HPP_
