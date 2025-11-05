cmake_minimum_required(VERSION 3.10)
project(pose_uncertainty_monitor)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tier4_debug_msgs REQUIRED)
find_package(autoware_localization_msgs REQUIRED)

# Add the library for the node
add_library(
  pose_uncertainty_monitor_node
  src/pose_uncertainty_monitor_node.cpp
)

# Add include directories for the library
target_include_directories(
  pose_uncertainty_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies to the library
ament_target_dependencies(
  pose_uncertainty_monitor_node
  PUBLIC
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "tier4_debug_msgs"
  "autoware_localization_msgs"
)

# Add the executable
add_executable(
  pose_uncertainty_monitor_exe
  src/main.cpp
)

# Link the executable to the library
target_link_libraries(
  pose_uncertainty_monitor_exe
  pose_uncertainty_monitor_node
)

# Install targets
install(
  TARGETS
  pose_uncertainty_monitor_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
