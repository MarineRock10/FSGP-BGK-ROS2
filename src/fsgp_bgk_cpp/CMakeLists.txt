cmake_minimum_required(VERSION 3.8)
project(fsgp_bgk_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(fsgp_bgk_msgs REQUIRED)

# Third-party
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Eigen3 REQUIRED)

# === Point cloud preprocessor library ===
add_library(point_cloud_preprocessor SHARED
  src/point_cloud_preprocessor.cpp
)
target_include_directories(point_cloud_preprocessor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(point_cloud_preprocessor
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)
ament_target_dependencies(point_cloud_preprocessor
  rclcpp sensor_msgs pcl_conversions
)

# === Global map manager library ===
add_library(global_map_manager SHARED
  src/global_map_manager.cpp
)
target_include_directories(global_map_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(global_map_manager
  Eigen3::Eigen
)
ament_target_dependencies(global_map_manager
  rclcpp sensor_msgs
)

# === Main traversability node ===
add_executable(fsgp_bgk_node
  src/fsgp_bgk_node.cpp
)
target_include_directories(fsgp_bgk_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(fsgp_bgk_node
  point_cloud_preprocessor
  global_map_manager
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)
ament_target_dependencies(fsgp_bgk_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
  fsgp_bgk_msgs
)

# Install targets
# Shared libraries go to lib/ so the dynamic linker can find them.
# The executable goes to lib/${PROJECT_NAME} (ROS 2 convention).
install(TARGETS
  point_cloud_preprocessor
  global_map_manager
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(TARGETS
  fsgp_bgk_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
