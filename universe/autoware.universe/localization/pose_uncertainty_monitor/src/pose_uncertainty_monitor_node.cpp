#include "pose_uncertainty_monitor/pose_uncertainty_monitor_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <cmath>

namespace pose_uncertainty_monitor
{

PoseUncertaintyMonitorNode::PoseUncertaintyMonitorNode(const rclcpp::NodeOptions & options)
: Node("pose_uncertainty_monitor", options)
{
  // Parameters
  const auto input_pose_with_cov_topic = declare_parameter<std::string>("input_pose_with_cov_topic");
  const auto output_topic = declare_parameter<std::string>("output_topic");

  // Subscriber
  sub_pose_with_cov_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    input_pose_with_cov_topic, rclcpp::QoS{1},
    std::bind(&PoseUncertaintyMonitorNode::on_pose_with_cov, this, std::placeholders::_1));

  // Publisher for the new vector type
  pub_uncertainty_vector_ = create_publisher<autoware_localization_msgs::msg::PoseUncertaintyVector>(
    output_topic, rclcpp::QoS{1});
}

void PoseUncertaintyMonitorNode::on_pose_with_cov(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  // Extract variances for x, y, and yaw from the 6x6 covariance matrix
  // Indices: 0(X-X), 7(Y-Y), 35(Yaw-Yaw)
  const double var_x = msg->pose.covariance[0];
  const double var_y = msg->pose.covariance[7];
  const double var_yaw = msg->pose.covariance[35];

  // Calculate standard deviations
  const double std_x = std::sqrt(var_x);
  const double std_y = std::sqrt(var_y);
  const double std_yaw = std::sqrt(var_yaw);

  // Create and publish the PoseUncertaintyVector message
  autoware_localization_msgs::msg::PoseUncertaintyVector uncertainty_msg;
  uncertainty_msg.std_x = std_x;
  uncertainty_msg.std_y = std_y;
  uncertainty_msg.std_yaw = std_yaw;

  pub_uncertainty_vector_->publish(uncertainty_msg);
}

}  // namespace pose_uncertainty_monitor

RCLCPP_COMPONENTS_REGISTER_NODE(pose_uncertainty_monitor::PoseUncertaintyMonitorNode)