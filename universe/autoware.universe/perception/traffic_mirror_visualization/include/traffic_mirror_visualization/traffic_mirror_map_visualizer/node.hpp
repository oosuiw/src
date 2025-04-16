// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TRAFFIC_MIRROR_VISUALIZATION__TRAFFIC_MIRROR_MAP_VISUALIZER__NODE_HPP_
#define TRAFFIC_MIRROR_VISUALIZATION__TRAFFIC_MIRROR_MAP_VISUALIZER__NODE_HPP_

#include <lanelet2_extension/regulatory_elements/autoware_traffic_mirror.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace traffic_mirror
{
class TrafficMirrorMapVisualizerNode : public rclcpp::Node
{
public:
  TrafficMirrorMapVisualizerNode(const std::string & node_name, const rclcpp::NodeOptions & options);
  ~TrafficMirrorMapVisualizerNode() = default;
  void trafficSignalsCallback(
    const autoware_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr
      input_traffic_signals_msg);
  void binMapCallback(
    const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr input_map_msg);

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mirror_marker_pub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::TrafficSignalArray>::SharedPtr tm_state_sub_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr vector_map_sub_;

  std::vector<lanelet::AutowareTrafficMirrorConstPtr> aw_tm_reg_elems_;
};

}  // namespace traffic_mirror

#endif  // TRAFFIC_MIRROR_VISUALIZATION__TRAFFIC_MIRROR_MAP_VISUALIZER__NODE_HPP_
