// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0

#include "nav2_constrained_controller/parameter_handler.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_constrained_controller
{

using nav2_util::declare_parameter_if_not_declared;

ParameterHandler::ParameterHandler(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
  const std::string & plugin_name,
  rclcpp::Logger logger)
: plugin_name_(plugin_name), logger_(logger)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("ParameterHandler: parent node has expired");
  }

  // Helper macros to keep the declaration block compact.
#define DECL_DOUBLE(name, default_val)                                        \
  declare_parameter_if_not_declared(                                          \
    node, plugin_name_ + "." #name, rclcpp::ParameterValue(default_val));     \
  node->get_parameter(plugin_name_ + "." #name, params_.name)
#define DECL_INT(name, default_val)                                           \
  declare_parameter_if_not_declared(                                          \
    node, plugin_name_ + "." #name, rclcpp::ParameterValue(default_val));    \
  node->get_parameter(plugin_name_ + "." #name, params_.name)
#define DECL_BOOL(name, default_val)                                          \
  declare_parameter_if_not_declared(                                          \
    node, plugin_name_ + "." #name, rclcpp::ParameterValue(default_val));    \
  node->get_parameter(plugin_name_ + "." #name, params_.name)
#define DECL_STRING(name, default_val)                                        \
  declare_parameter_if_not_declared(                                          \
    node, plugin_name_ + "." #name,                                           \
    rclcpp::ParameterValue(std::string(default_val)));                        \
  node->get_parameter(plugin_name_ + "." #name, params_.name)

  DECL_DOUBLE(v_linear_min, 0.05);
  DECL_DOUBLE(v_linear_max, 0.4);
  DECL_DOUBLE(v_lateral_max, 0.3);
  DECL_DOUBLE(v_angular_max, 1.5);

  DECL_DOUBLE(slowdown_radius, 0.6);
  DECL_DOUBLE(k_yaw, 1.5);
  DECL_BOOL(yaw_correction_ramp, true);

  DECL_DOUBLE(motion_target_dist, 0.4);
  DECL_DOUBLE(max_robot_pose_search_dist, 2.0);
  DECL_DOUBLE(goal_dist_tolerance, 0.05);

  DECL_DOUBLE(footprint_length, 0.65);
  DECL_DOUBLE(footprint_dl, 0.05);
  DECL_DOUBLE(footprint_db, 0.30);

  DECL_DOUBLE(cbf_gamma, 2.0);
  DECL_DOUBLE(alley_width_min, 0.95);
  DECL_DOUBLE(alley_width_max, 1.10);
  DECL_DOUBLE(alley_width_tol, 0.10);
  DECL_DOUBLE(wall_consideration_range, 2.5);
  DECL_BOOL(enable_inner_corner_cbf, false);

  DECL_STRING(lidar_topic, "/scan");
  DECL_DOUBLE(lidar_max_range, 8.0);
  DECL_DOUBLE(lidar_min_range, 0.05);
  DECL_DOUBLE(line_split_threshold, 0.04);
  DECL_INT(line_min_points, 8);
  DECL_DOUBLE(segment_connect_dist, 0.12);
  DECL_DOUBLE(parallel_cos_tol, 0.95);
  DECL_DOUBLE(perp_cos_tol, 0.20);

  DECL_BOOL(enable_lateral_centering, true);
  DECL_DOUBLE(k_lat, 1.0);
  DECL_DOUBLE(flanking_cos_tol, 0.94);
  DECL_DOUBLE(target_half_width, 0.5125);
  DECL_DOUBLE(max_centering_range, 1.1);
  DECL_BOOL(require_both_walls, true);
  DECL_INT(regime_switch_hyst_ticks, 3);
  DECL_DOUBLE(vy_centering_lpf_alpha, 0.4);

  DECL_STRING(log_dir, "/root/navigation_log");
  DECL_BOOL(log_enabled, true);
  DECL_INT(log_lidar_every_n_ticks, 10);

#undef DECL_DOUBLE
#undef DECL_INT
#undef DECL_BOOL
#undef DECL_STRING

  params_.v_linear_max_initial = params_.v_linear_max;
  params_.v_angular_max_initial = params_.v_angular_max;

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ParameterHandler::dynamicParametersCallback,
      this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
ParameterHandler::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & p : parameters) {
    const auto & name = p.get_name();
    const auto type = p.get_type();
    if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      const double v = p.as_double();
      if (name == plugin_name_ + ".v_linear_min") {params_.v_linear_min = v;}
      else if (name == plugin_name_ + ".v_linear_max") {
        params_.v_linear_max = v; params_.v_linear_max_initial = v;
      } else if (name == plugin_name_ + ".v_lateral_max") {
        params_.v_lateral_max = v;
      } else if (name == plugin_name_ + ".v_angular_max") {
        params_.v_angular_max = v; params_.v_angular_max_initial = v;
      } else if (name == plugin_name_ + ".slowdown_radius") {
        params_.slowdown_radius = v;
      } else if (name == plugin_name_ + ".k_yaw") {params_.k_yaw = v;}
      else if (name == plugin_name_ + ".motion_target_dist") {
        params_.motion_target_dist = v;
      } else if (name == plugin_name_ + ".cbf_gamma") {params_.cbf_gamma = v;}
      else if (name == plugin_name_ + ".footprint_length") {
        params_.footprint_length = v;
      } else if (name == plugin_name_ + ".footprint_dl") {
        params_.footprint_dl = v;
      } else if (name == plugin_name_ + ".footprint_db") {
        params_.footprint_db = v;
      } else if (name == plugin_name_ + ".k_lat") {
        params_.k_lat = v;
      } else if (name == plugin_name_ + ".flanking_cos_tol") {
        params_.flanking_cos_tol = v;
      } else if (name == plugin_name_ + ".target_half_width") {
        params_.target_half_width = v;
      } else if (name == plugin_name_ + ".vy_centering_lpf_alpha") {
        params_.vy_centering_lpf_alpha = v;
      } else if (name == plugin_name_ + ".max_centering_range") {
        params_.max_centering_range = v;
      }
    } else if (type == rclcpp::ParameterType::PARAMETER_BOOL) {
      const bool v = p.as_bool();
      if (name == plugin_name_ + ".yaw_correction_ramp") {
        params_.yaw_correction_ramp = v;
      } else if (name == plugin_name_ + ".log_enabled") {
        params_.log_enabled = v;
      } else if (name == plugin_name_ + ".enable_lateral_centering") {
        params_.enable_lateral_centering = v;
      } else if (name == plugin_name_ + ".require_both_walls") {
        params_.require_both_walls = v;
      }
    } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
      const int v = static_cast<int>(p.as_int());
      if (name == plugin_name_ + ".line_min_points") {
        params_.line_min_points = v;
      } else if (name == plugin_name_ + ".log_lidar_every_n_ticks") {
        params_.log_lidar_every_n_ticks = v;
      } else if (name == plugin_name_ + ".regime_switch_hyst_ticks") {
        params_.regime_switch_hyst_ticks = v;
      }
    }
  }
  return result;
}

}  // namespace nav2_constrained_controller
