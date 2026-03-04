/*
 * MIT License
 * Copyright (c) 2025 Senming Tan
 */

#include "fsgp_bgk_cpp/point_cloud_preprocessor.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace fsgp_bgk
{

PointCloudPreprocessor::PointCloudPreprocessor(const PreprocessorConfig & config)
: config_(config)
{
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
PointCloudPreprocessor::filter_cloud(const sensor_msgs::msg::PointCloud2 & msg) const
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(msg, *cloud);

  auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  filtered->reserve(cloud->size());

  const double max_radius_sq = config_.max_radius * config_.max_radius;

  for (const auto & pt : cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }

    // Height filter
    if (pt.z >= config_.max_height) {
      continue;
    }

    // Radius filter (in XY plane)
    const double radius_sq = pt.x * pt.x + pt.y * pt.y;
    if (radius_sq >= max_radius_sq) {
      continue;
    }

    filtered->points.push_back(pt);
  }

  filtered->width = filtered->points.size();
  filtered->height = 1;
  filtered->is_dense = true;

  return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
PointCloudPreprocessor::voxel_downsample(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) const
{
  auto downsampled = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(
    static_cast<float>(config_.voxel_size),
    static_cast<float>(config_.voxel_size),
    static_cast<float>(config_.voxel_size));
  voxel_filter.filter(*downsampled);

  return downsampled;
}

std::vector<float>
PointCloudPreprocessor::preprocess(
  const sensor_msgs::msg::PointCloud2 & msg,
  uint32_t & num_points) const
{
  // Filter by height and radius
  auto filtered = filter_cloud(msg);

  // Voxel downsample
  auto downsampled = voxel_downsample(filtered);

  num_points = static_cast<uint32_t>(downsampled->points.size());

  // Flatten to [x0,y0,z0, x1,y1,z1, ...]
  std::vector<float> flat_points;
  flat_points.reserve(num_points * 3);

  for (const auto & pt : downsampled->points) {
    flat_points.push_back(pt.x);
    flat_points.push_back(pt.y);
    flat_points.push_back(pt.z);
  }

  return flat_points;
}

void PointCloudPreprocessor::update_config(const PreprocessorConfig & config)
{
  config_ = config;
}

}  // namespace fsgp_bgk
