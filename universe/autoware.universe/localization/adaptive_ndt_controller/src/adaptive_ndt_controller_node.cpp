#include "adaptive_ndt_controller/adaptive_ndt_controller_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <algorithm>
#include <chrono>
#include <vector>

namespace adaptive_ndt_controller
{

// Helper function for linear interpolation
_Float64 lerp(double x, double x0, double x1, double y0, double y1)
{
  if (x <= x0) return y0;
  if (x >= x1) return y1;
  return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

AdaptiveNdtControllerNode::AdaptiveNdtControllerNode(const rclcpp::NodeOptions & options)
: Node("adaptive_ndt_controller", options), params_initialized_(false)
{
  // Declare and get parameters
  ndt_node_name_ = declare_parameter<std::string>("ndt_node_name");
  uncertainty_topic_ = declare_parameter<std::string>("uncertainty_topic");

  pos_uncertainty_min_threshold_ = declare_parameter<double>("pos_uncertainty_min_threshold");
  pos_uncertainty_max_threshold_ = declare_parameter<double>("pos_uncertainty_max_threshold");
  yaw_uncertainty_min_threshold_ = declare_parameter<double>("yaw_uncertainty_min_threshold");
  yaw_uncertainty_max_threshold_ = declare_parameter<double>("yaw_uncertainty_max_threshold");

  enable_adaptive_step_size_ = declare_parameter<bool>("enable_adaptive_step_size");
  step_size_min_ = declare_parameter<double>("step_size_min");
  step_size_max_ = declare_parameter<double>("step_size_max");

  enable_adaptive_max_iterations_ = declare_parameter<bool>("enable_adaptive_max_iterations");
  max_iterations_min_ = declare_parameter<int>("max_iterations_min");
  max_iterations_max_ = declare_parameter<int>("max_iterations_max");

  enable_adaptive_resolution_ = declare_parameter<bool>("enable_adaptive_resolution");
  resolution_min_ = declare_parameter<double>("resolution_min");
  resolution_max_ = declare_parameter<double>("resolution_max");

  // Adaptive range control parameters
  crop_box_node_name_ = declare_parameter<std::string>("crop_box_node_name");
  enable_adaptive_range_control_ = declare_parameter<bool>("enable_adaptive_range_control");
  range_pos_uncertainty_threshold_ = declare_parameter<double>("range_pos_uncertainty_threshold");
  base_max_x_ = declare_parameter<double>("base_max_x");
  base_max_y_ = declare_parameter<double>("base_max_y");
  expanded_max_x_ = declare_parameter<double>("expanded_max_x");
  expanded_max_y_ = declare_parameter<double>("expanded_max_y");

  // Initialize parameter clients and subscriber
  ndt_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, ndt_node_name_);
  crop_box_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, crop_box_node_name_);
  sub_uncertainty_vector_ = create_subscription<autoware_localization_msgs::msg::PoseUncertaintyVector>(
    uncertainty_topic_, rclcpp::QoS{1},
    std::bind(&AdaptiveNdtControllerNode::on_uncertainty_vector, this, std::placeholders::_1));
}

void AdaptiveNdtControllerNode::on_uncertainty_vector(
  const autoware_localization_msgs::msg::PoseUncertaintyVector::ConstSharedPtr msg)
{
  update_ndt_parameters(msg->std_x, msg->std_y, msg->std_yaw);
}

void AdaptiveNdtControllerNode::update_ndt_parameters(
  double std_x, double std_y, double std_yaw)
{
  std::vector<rclcpp::Parameter> new_params;
  double position_uncertainty = std::sqrt(std_x * std_x + std_y * std_y);

  // === Adaptive Step Size Control ===
  if (enable_adaptive_step_size_) {
    double new_step_size = lerp(
      position_uncertainty, pos_uncertainty_min_threshold_, pos_uncertainty_max_threshold_,
      step_size_min_, step_size_max_);

    if (!params_initialized_ || std::abs(new_step_size - last_step_size_) > 1e-6) {
      new_params.push_back(rclcpp::Parameter("step_size", new_step_size));
      last_step_size_ = new_step_size;
      RCLCPP_INFO(
        this->get_logger(), "Adaptive step_size: %.4f (pos_uncertainty=%.4f)",
        new_step_size, position_uncertainty);
    }
  }

  // === Adaptive Max Iterations Control ===
  if (enable_adaptive_max_iterations_) {
    int new_max_iterations = static_cast<int>(lerp(
      std_yaw, yaw_uncertainty_min_threshold_, yaw_uncertainty_max_threshold_,
      max_iterations_min_, max_iterations_max_));

    if (!params_initialized_ || new_max_iterations != last_max_iterations_) {
      new_params.push_back(rclcpp::Parameter("max_iterations", new_max_iterations));
      last_max_iterations_ = new_max_iterations;
      RCLCPP_INFO(
        this->get_logger(), "Adaptive max_iterations: %d (std_yaw=%.4f)",
        new_max_iterations, std_yaw);
    }
  }

  // === Adaptive Resolution Control ===
  if (enable_adaptive_resolution_) {
    double new_resolution = lerp(
      position_uncertainty, pos_uncertainty_min_threshold_, pos_uncertainty_max_threshold_,
      resolution_min_, resolution_max_);

    if (!params_initialized_ || std::abs(new_resolution - last_resolution_) > 1e-6) {
      new_params.push_back(rclcpp::Parameter("resolution", new_resolution));
      last_resolution_ = new_resolution;
      RCLCPP_INFO(
        this->get_logger(), "Adaptive resolution: %.4f (pos_uncertainty=%.4f)",
        new_resolution, position_uncertainty);
    }
  }

  // === Adaptive Measurement Range Control ===
  if (enable_adaptive_range_control_) {
    double target_max_x = base_max_x_;
    double target_max_y = base_max_y_;

    if (position_uncertainty > range_pos_uncertainty_threshold_) {
      target_max_x = expanded_max_x_;
      target_max_y = expanded_max_y_;
    }

    std::vector<rclcpp::Parameter> crop_box_params;
    if (!params_initialized_ || std::abs(target_max_x - last_max_x_) > 1e-6) {
      crop_box_params.push_back(rclcpp::Parameter("max_x", target_max_x));
      crop_box_params.push_back(rclcpp::Parameter("min_x", -target_max_x)); // Assuming symmetric range
      last_max_x_ = target_max_x;
    }
    if (!params_initialized_ || std::abs(target_max_y - last_max_y_) > 1e-6) {
      crop_box_params.push_back(rclcpp::Parameter("max_y", target_max_y));
      crop_box_params.push_back(rclcpp::Parameter("min_y", -target_max_y)); // Assuming symmetric range
      last_max_y_ = target_max_y;
    }

    if (!crop_box_params.empty()) {
      RCLCPP_INFO(this->get_logger(), "Adapting measurement range: max_x=%.1f, max_y=%.1f", target_max_x, target_max_y);
      if (crop_box_param_client_->service_is_ready()) {
        crop_box_param_client_->set_parameters(crop_box_params);
      } else {
        RCLCPP_WARN(this->get_logger(), "Crop Box Filter parameter service is not available.");
      }
    }
  }

  // Send parameters to NDT node if there are changes
  if (!new_params.empty()) {
    if (!ndt_param_client_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "NDT parameter service is not available.");
      return;
    }
    ndt_param_client_->set_parameters(new_params);
    params_initialized_ = true;
  }
}

}  // namespace adaptive_ndt_controller

RCLCPP_COMPONENTS_REGISTER_NODE(adaptive_ndt_controller::AdaptiveNdtControllerNode)
