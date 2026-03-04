/*
 * MIT License
 * Copyright (c) 2025 Senming Tan
 *
 * Main FSGP-BGK ROS2 node. Orchestrates:
 * - Point cloud subscription and preprocessing (C++)
 * - Traversability computation via Python service call
 * - Global map management and publishing (C++)
 */

#ifndef FSGP_BGK_CPP__FSGP_BGK_NODE_HPP_
#define FSGP_BGK_CPP__FSGP_BGK_NODE_HPP_

#include <memory>
#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "fsgp_bgk_msgs/srv/compute_traversability.hpp"
#include "fsgp_bgk_msgs/msg/traversability_grid.hpp"
#include "fsgp_bgk_cpp/point_cloud_preprocessor.hpp"
#include "fsgp_bgk_cpp/global_map_manager.hpp"

namespace fsgp_bgk
{

class FsgpBgkNode : public rclcpp::Node
{
public:
  explicit FsgpBgkNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ROS parameters
  void declare_and_load_parameters();

  // Callbacks
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void map_update_timer_callback();

  // Service call to Python algorithm
  void call_traversability_service(
    const std::array<double, 6> & pose,
    const std::vector<float> & points,
    uint32_t num_points);

  // Handle service response
  void on_traversability_response(
    rclcpp::Client<fsgp_bgk_msgs::srv::ComputeTraversability>::SharedFuture future);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traversability_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traversability_ldd_pub_;
  rclcpp::Publisher<fsgp_bgk_msgs::msg::TraversabilityGrid>::SharedPtr grid_pub_;

  // Service client
  rclcpp::Client<fsgp_bgk_msgs::srv::ComputeTraversability>::SharedPtr traversability_client_;

  // Timer
  rclcpp::TimerBase::SharedPtr map_timer_;

  // Components
  std::unique_ptr<PointCloudPreprocessor> preprocessor_;
  std::unique_ptr<GlobalMapManager> map_manager_;

  // State
  std::array<double, 6> current_pose_{};
  bool pose_received_ = false;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  bool service_in_progress_ = false;

  // Parameters
  std::string cloud_topic_;
  std::string odom_topic_;
  std::string global_frame_;
  double map_update_rate_;
};

}  // namespace fsgp_bgk

#endif  // FSGP_BGK_CPP__FSGP_BGK_NODE_HPP_
