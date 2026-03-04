/*
 * MIT License
 * Copyright (c) 2025 Senming Tan
 *
 * Main FSGP-BGK C++ ROS2 node.
 *
 * This node handles the engineering infrastructure:
 * - Subscribes to point cloud and odometry
 * - Preprocesses point clouds (voxel downsample, filtering)
 * - Calls the Python traversability service for core algorithm
 * - Manages the global traversability map
 * - Publishes traversability point clouds
 *
 * The Python algorithm core remains unchanged.
 */

#include "fsgp_bgk_cpp/fsgp_bgk_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <chrono>
#include <functional>

namespace fsgp_bgk
{

FsgpBgkNode::FsgpBgkNode(const rclcpp::NodeOptions & options)
: Node("fsgp_bgk_node", options)
{
  declare_and_load_parameters();

  // Initialize components
  PreprocessorConfig preproc_config;
  preproc_config.voxel_size = this->get_parameter("downsampl_voxel_size").as_double();
  preproc_config.max_height = this->get_parameter("max_cloud_height").as_double();

  const double x_length = this->get_parameter("x_length").as_double();
  const double y_length = this->get_parameter("y_length").as_double();
  preproc_config.max_radius = (x_length + y_length) / 4.0;

  preprocessor_ = std::make_unique<PointCloudPreprocessor>(preproc_config);

  GlobalMapConfig map_config;
  map_config.grid_resolution = this->get_parameter("resolution").as_double();
  map_config.publish_resolution = this->get_parameter("publish_resolution").as_double();
  map_config.max_radius = preproc_config.max_radius;
  map_config.occupancy_threshold = this->get_parameter("occupancy_threshold").as_double();
  map_config.obstacle_threshold = this->get_parameter("obstacle_threshold").as_double();
  map_config.smooth_kernel_size = this->get_parameter("smooth_kernel_size").as_int();
  map_config.binarization_condition = this->get_parameter("binarization_condition").as_bool();

  map_manager_ = std::make_unique<GlobalMapManager>(map_config);

  // Subscribers
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, 3,
    std::bind(&FsgpBgkNode::pointcloud_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 3,
    std::bind(&FsgpBgkNode::odom_callback, this, std::placeholders::_1));

  // Publishers
  traversability_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "traversability_pcl", 3);
  traversability_ldd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "traversability_pcl_ldd", 3);
  grid_pub_ = this->create_publisher<fsgp_bgk_msgs::msg::TraversabilityGrid>(
    "traversability_grid", 3);

  // Service client
  traversability_client_ = this->create_client<fsgp_bgk_msgs::srv::ComputeTraversability>(
    "compute_traversability");

  // Map update timer
  map_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(map_update_rate_),
    std::bind(&FsgpBgkNode::map_update_timer_callback, this));

  RCLCPP_INFO(this->get_logger(),
    "FSGP-BGK C++ node initialized. Cloud: %s, Odom: %s, Rate: %.3fs",
    cloud_topic_.c_str(), odom_topic_.c_str(), map_update_rate_);
}

void FsgpBgkNode::declare_and_load_parameters()
{
  // ROS topics
  this->declare_parameter<std::string>("cloud_topic", "/simulated_lidar_local");
  this->declare_parameter<std::string>("odom_topic", "/odom");
  this->declare_parameter<std::string>("global_link", "map");

  // Map parameters
  this->declare_parameter<double>("max_cloud_height", 0.5);
  this->declare_parameter<double>("downsampl_voxel_size", 0.3);
  this->declare_parameter<bool>("binarization_condition", true);
  this->declare_parameter<double>("obstacle_threshold", 0.5);
  this->declare_parameter<double>("occupancy_threshold", 0.5);
  this->declare_parameter<int>("smooth_kernel_size", 3);
  this->declare_parameter<double>("map_update_rate", 0.025);
  this->declare_parameter<double>("publish_resolution", 0.1);

  // Grid parameters
  this->declare_parameter<double>("resolution", 0.2);
  this->declare_parameter<double>("x_length", 5.0);
  this->declare_parameter<double>("y_length", 5.0);

  // Load values
  cloud_topic_ = this->get_parameter("cloud_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  global_frame_ = this->get_parameter("global_link").as_string();
  map_update_rate_ = this->get_parameter("map_update_rate").as_double();
}

void FsgpBgkNode::pointcloud_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  latest_cloud_ = msg;
}

void FsgpBgkNode::odom_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & pos = msg->pose.pose.position;
  const auto & ori = msg->pose.pose.orientation;

  tf2::Quaternion q(ori.x, ori.y, ori.z, ori.w);
  tf2::Matrix3x3 mat(q);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  current_pose_ = {pos.x, pos.y, pos.z, roll, pitch, yaw};
  pose_received_ = true;
}

void FsgpBgkNode::map_update_timer_callback()
{
  if (!latest_cloud_ || !pose_received_) {
    return;
  }

  if (service_in_progress_) {
    RCLCPP_DEBUG(this->get_logger(), "Service call still in progress, skipping this cycle");
    return;
  }

  if (!traversability_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Waiting for compute_traversability service...");
    return;
  }

  // Preprocess point cloud
  uint32_t num_points = 0;
  auto flat_points = preprocessor_->preprocess(*latest_cloud_, num_points);

  if (num_points < static_cast<uint32_t>(preprocessor_->config().min_points_threshold)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Too few points after preprocessing: %u", num_points);
    return;
  }

  // Call Python service
  call_traversability_service(current_pose_, flat_points, num_points);
}

void FsgpBgkNode::call_traversability_service(
  const std::array<double, 6> & pose,
  const std::vector<float> & points,
  uint32_t num_points)
{
  auto request = std::make_shared<fsgp_bgk_msgs::srv::ComputeTraversability::Request>();

  for (size_t i = 0; i < 6; ++i) {
    request->pose[i] = pose[i];
  }
  request->points = points;
  request->num_points = num_points;

  service_in_progress_ = true;

  auto future = traversability_client_->async_send_request(
    request,
    std::bind(&FsgpBgkNode::on_traversability_response, this, std::placeholders::_1));
}

void FsgpBgkNode::on_traversability_response(
  rclcpp::Client<fsgp_bgk_msgs::srv::ComputeTraversability>::SharedFuture future)
{
  service_in_progress_ = false;

  try {
    auto response = future.get();

    if (!response->success) {
      RCLCPP_WARN(this->get_logger(), "Traversability service failed: %s",
        response->message.c_str());
      return;
    }

    // Build TraversabilityResult from service response
    TraversabilityResult result;
    const size_t n = response->num_grid_points;

    result.grid_x.resize(n);
    result.grid_y.resize(n);
    result.mean_heights = response->mean_heights;
    result.traversability = response->traversability;

    for (size_t i = 0; i < n; ++i) {
      result.grid_x[i] = response->grid_points[i * 2];
      result.grid_y[i] = response->grid_points[i * 2 + 1];
    }

    // Update global map
    map_manager_->update(current_pose_, result);

    // Publish TraversabilityGrid message
    fsgp_bgk_msgs::msg::TraversabilityGrid grid_msg;
    grid_msg.header.stamp = this->now();
    grid_msg.header.frame_id = global_frame_;
    grid_msg.resolution = static_cast<float>(
      this->get_parameter("resolution").as_double());
    grid_msg.x_length = static_cast<float>(
      this->get_parameter("x_length").as_double());
    grid_msg.y_length = static_cast<float>(
      this->get_parameter("y_length").as_double());
    for (size_t i = 0; i < 6; ++i) {
      grid_msg.pose[i] = current_pose_[i];
    }
    grid_msg.num_cells = static_cast<uint32_t>(n);
    grid_msg.grid_x = result.grid_x;
    grid_msg.grid_y = result.grid_y;
    grid_msg.mean_heights = result.mean_heights;
    grid_msg.traversability = result.traversability;
    grid_pub_->publish(grid_msg);

    // Generate and publish output point clouds
    sensor_msgs::msg::PointCloud2 raw_cloud, ldd_cloud;
    map_manager_->generate_output_clouds(
      current_pose_, global_frame_, this->now(), raw_cloud, ldd_cloud);

    traversability_pub_->publish(raw_cloud);
    traversability_ldd_pub_->publish(ldd_cloud);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing traversability response: %s", e.what());
  }
}

}  // namespace fsgp_bgk

// Main entry point
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fsgp_bgk::FsgpBgkNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
