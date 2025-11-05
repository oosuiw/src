cmake_minimum_required(VERSION 3.10)
project(adaptive_ndt_controller)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(autoware_localization_msgs REQUIRED)

# Add the library for the node (SHARED library for component)
add_library(
  adaptive_ndt_controller_node SHARED
  src/adaptive_ndt_controller_node.cpp
)

# Add include directories for the library
target_include_directories(
  adaptive_ndt_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies to the library
ament_target_dependencies(
  adaptive_ndt_controller_node
  PUBLIC
  "rclcpp"
  "rclcpp_components"
  "autoware_localization_msgs"
)

# Register the node as a component (this also creates the executable)
rclcpp_components_register_node(
  adaptive_ndt_controller_node
  PLUGIN "adaptive_ndt_controller::AdaptiveNdtControllerNode"
  EXECUTABLE adaptive_ndt_controller_exe
)

# Install targets
install(
  TARGETS adaptive_ndt_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS adaptive_ndt_controller_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
