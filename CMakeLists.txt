cmake_minimum_required(VERSION 3.5)
project(ioo_ros2)

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_ioo_ros2 SHARED
  DIRECTORY src
)
include_directories(${EIGEN3_INCLUDE_DIR})
target_include_directories(target_ioo_ros2
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}
)

rclcpp_components_register_node(target_ioo_ros2
  PLUGIN "ioo_ros2::ImuOnlyOdometryROS2"
  EXECUTABLE ioo_ros2_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)