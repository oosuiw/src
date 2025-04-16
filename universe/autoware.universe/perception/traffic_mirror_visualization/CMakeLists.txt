cmake_minimum_required(VERSION 3.14)
project(traffic_mirror_visualization)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)

ament_auto_add_library(traffic_mirror_roi_visualizer_nodelet SHARED
  src/traffic_mirror_roi_visualizer/nodelet.cpp
)

target_link_libraries(traffic_mirror_roi_visualizer_nodelet
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(traffic_mirror_roi_visualizer_nodelet
  PLUGIN "traffic_mirror::TrafficMirrorRoiVisualizerNodelet"
  EXECUTABLE traffic_mirror_visualization_node
)

ament_auto_add_executable(traffic_mirror_map_visualizer_node
  src/traffic_mirror_map_visualizer/node.cpp
  src/traffic_mirror_map_visualizer/main.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
