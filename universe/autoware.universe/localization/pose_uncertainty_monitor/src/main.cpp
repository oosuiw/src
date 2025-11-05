#include "pose_uncertainty_monitor/pose_uncertainty_monitor_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<pose_uncertainty_monitor::PoseUncertaintyMonitorNode>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
