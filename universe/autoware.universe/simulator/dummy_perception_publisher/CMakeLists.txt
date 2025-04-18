cmake_minimum_required(VERSION 3.14)  #KMS_250317
project(dummy_perception_publisher)  #KMS_250317

find_package(autoware_cmake REQUIRED)  #KMS_250317
autoware_package()  #KMS_250317

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InitialState.msg"
  "msg/Object.msg"
  DEPENDENCIES autoware_auto_perception_msgs tier4_perception_msgs geometry_msgs std_msgs unique_identifier_msgs
)  #KMS_250317

# See ndt_omp package for documentation on why PCL is special
find_package(PCL REQUIRED COMPONENTS common filters)  #KMS_250317

set(${PROJECT_NAME}_DEPENDENCIES
  autoware_auto_perception_msgs
  tier4_perception_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)  #KMS_250317

ament_auto_add_library(signed_distance_function SHARED
  src/signed_distance_function.cpp
)  #KMS_250317

ament_auto_add_executable(dummy_perception_publisher_node
  src/main.cpp
  src/node.cpp
  src/pointcloud_creator.cpp
)  #KMS_250317

target_link_libraries(dummy_perception_publisher_node
  signed_distance_function
)  #KMS_250317

# 기존 signed_distance_function_lib 관련 find_library 및 링크 구문은 제거함  #KMS_250317

ament_target_dependencies(dummy_perception_publisher_node ${${PROJECT_NAME}_DEPENDENCIES})  #KMS_250317

target_include_directories(dummy_perception_publisher_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)  #KMS_250317

# For using message definitions from the same package
if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
  rosidl_target_interfaces(dummy_perception_publisher_node
    ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )  #KMS_250317
else()
  rosidl_get_typesupport_target(
    cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )  #KMS_250317
  target_link_libraries(dummy_perception_publisher_node "${cpp_typesupport_target}")  #KMS_250317
endif()

# PCL dependencies - `ament_target_dependencies` doesn't respect the
# components/modules selected above and only links in `common` ,so we need
# to do this manually.
target_compile_definitions(dummy_perception_publisher_node PRIVATE ${PCL_DEFINITIONS})  #KMS_250317
target_include_directories(dummy_perception_publisher_node PRIVATE ${PCL_INCLUDE_DIRS})  #KMS_250317
# Unfortunately, this one can't be PRIVATE because only the plain or only the
# keyword (PRIVATE) signature of target_link_libraries can be used for one
# target, not both. The plain signature is already used inside
# `ament_target_dependencies` and possibly rosidl_target_interfaces.
target_link_libraries(dummy_perception_publisher_node ${PCL_LIBRARIES})  #KMS_250317
target_link_directories(dummy_perception_publisher_node PRIVATE ${PCL_LIBRARY_DIRS})  #KMS_250317

ament_auto_add_executable(empty_objects_publisher
  src/empty_objects_publisher.cpp
)  #KMS_250317

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(signed_distance_function-test
    test/src/test_signed_distance_function.cpp
  )  #KMS_250317
  target_link_libraries(signed_distance_function-test
    signed_distance_function
  )  #KMS_250317
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)  #KMS_250317
