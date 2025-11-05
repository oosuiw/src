#ifndef POSE_UNCERTAINTY_MONITOR__POSE_UNCERTAINTY_MONITOR_NODE_HPP_
#define POSE_UNCERTAINTY_MONITOR__POSE_UNCERTAINTY_MONITOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "autoware_localization_msgs/msg/pose_uncertainty_vector.hpp"

namespace pose_uncertainty_monitor
{

class PoseUncertaintyMonitorNode : public rclcpp::Node
{
public:
  explicit PoseUncertaintyMonitorNode(const rclcpp::NodeOptions & options);

private:
  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_with_cov_;

  // Publisher
  rclcpp::Publisher<autoware_localization_msgs::msg::PoseUncertaintyVector>::SharedPtr pub_uncertainty_vector_;

  // Callback
  void on_pose_with_cov(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
};

}  // namespace pose_uncertainty_monitor

#endif  // POSE_UNCERTAINTY_MONITOR__POSE_UNCERTAINTY_MONITOR_NODE_HPP_