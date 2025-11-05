#ifndef ADAPTIVE_NDT_CONTROLLER__ADAPTIVE_NDT_CONTROLLER_NODE_HPP_
#define ADAPTIVE_NDT_CONTROLLER__ADAPTIVE_NDT_CONTROLLER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "autoware_localization_msgs/msg/pose_uncertainty_vector.hpp"

#include <memory>
#include <string>
#include <vector>

namespace adaptive_ndt_controller
{

class AdaptiveNdtControllerNode : public rclcpp::Node
{
public:
  explicit AdaptiveNdtControllerNode(const rclcpp::NodeOptions & options);

private:
  // Subscriber
  rclcpp::Subscription<autoware_localization_msgs::msg::PoseUncertaintyVector>::SharedPtr
    sub_uncertainty_vector_;

  // Parameter client
  rclcpp::AsyncParametersClient::SharedPtr ndt_param_client_;
  rclcpp::AsyncParametersClient::SharedPtr crop_box_param_client_;

  // Node name
  std::string ndt_node_name_;
  std::string crop_box_node_name_;
  std::string uncertainty_topic_;

  // Uncertainty thresholds
  double pos_uncertainty_min_threshold_;
  double pos_uncertainty_max_threshold_;
  double yaw_uncertainty_min_threshold_;
  double yaw_uncertainty_max_threshold_;

  // Control parameters
  bool enable_adaptive_step_size_;
  double step_size_min_;
  double step_size_max_;

  bool enable_adaptive_max_iterations_;
  int max_iterations_min_;
  int max_iterations_max_;

  bool enable_adaptive_resolution_;
  double resolution_min_;
  double resolution_max_;

  bool enable_adaptive_range_control_;
  double range_pos_uncertainty_threshold_;
  double base_max_x_;
  double base_max_y_;
  double expanded_max_x_;
  double expanded_max_y_;

  // Cache of last applied parameters
  double last_step_size_;
  int last_max_iterations_;
  double last_resolution_;
  double last_max_x_;
  double last_max_y_;
  bool params_initialized_;

  // Callbacks
  void on_uncertainty_vector(
    const autoware_localization_msgs::msg::PoseUncertaintyVector::ConstSharedPtr msg);

  // Core adaptive control logic
  void update_ndt_parameters(
    double std_x, double std_y, double std_yaw);
};

}  // namespace adaptive_ndt_controller

#endif  // ADAPTIVE_NDT_CONTROLLER__ADAPTIVE_NDT_CONTROLLER_NODE_HPP_