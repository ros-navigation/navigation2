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

  DECL_DOUBLE(slowdown_radius, 0.20);
  DECL_DOUBLE(k_yaw, 1.5);
  DECL_DOUBLE(k_lat, 0.8);
  DECL_DOUBLE(motion_target_dist, 0.30);
  DECL_DOUBLE(max_robot_pose_search_dist, 2.0);

  DECL_DOUBLE(goal_dist_tolerance, 0.05);

  DECL_DOUBLE(footprint_length, 0.90);
  DECL_DOUBLE(footprint_dl, 0.05);
  DECL_DOUBLE(footprint_db, 0.375);

  DECL_DOUBLE(cbf_gamma, 1.5);
  DECL_DOUBLE(wall_slow_h_thresh, 0.25);
  DECL_INT(cbf_n_predict_steps, 2);
  DECL_DOUBLE(cbf_predict_dt, 0.10);
  DECL_DOUBLE(esdf_d_safe, 0.03);
  DECL_DOUBLE(wall_consideration_range, 2.5);
  DECL_DOUBLE(cbf_slack_weight, 100.0);

  DECL_STRING(pointcloud_topic, "/livox/amr/lidar");
  DECL_DOUBLE(esdf_z_min, 1.247);
  DECL_DOUBLE(esdf_z_max, 1.447);
  DECL_DOUBLE(esdf_grid_resolution, 0.01);
  DECL_DOUBLE(esdf_grid_size_m, 3.0);

  DECL_BOOL(cbf_retreat_enabled, true);
  DECL_DOUBLE(cbf_retreat_h_enter, 0.02);
  DECL_DOUBLE(cbf_retreat_h_exit, 0.05);
  DECL_DOUBLE(cbf_retreat_speed, 0.10);
  DECL_DOUBLE(cbf_retreat_rotation_speed, 0.30);
  DECL_DOUBLE(cbf_retreat_lookahead_s, 0.30);

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
      } else if (name == plugin_name_ + ".slowdown_radius") {params_.slowdown_radius = v;}
      else if (name == plugin_name_ + ".k_yaw") {params_.k_yaw = v;}
      else if (name == plugin_name_ + ".k_lat") {params_.k_lat = v;}
      else if (name == plugin_name_ + ".motion_target_dist") {params_.motion_target_dist = v;}
      else if (name == plugin_name_ + ".cbf_gamma") {params_.cbf_gamma = v;}
      else if (name == plugin_name_ + ".wall_slow_h_thresh") {
        params_.wall_slow_h_thresh = v;
      } else if (name == plugin_name_ + ".cbf_predict_dt") {
        params_.cbf_predict_dt = v;
      } else if (name == plugin_name_ + ".footprint_length") {
        params_.footprint_length = v;
      } else if (name == plugin_name_ + ".footprint_dl") {
        params_.footprint_dl = v;
      } else if (name == plugin_name_ + ".footprint_db") {
        params_.footprint_db = v;
      } else if (name == plugin_name_ + ".cbf_gamma") {params_.cbf_gamma = v;}
      else if (name == plugin_name_ + ".esdf_d_safe") {params_.esdf_d_safe = v;}
      else if (name == plugin_name_ + ".wall_consideration_range") {
        params_.wall_consideration_range = v;
      } else if (name == plugin_name_ + ".cbf_slack_weight") {
        params_.cbf_slack_weight = v;
      } else if (name == plugin_name_ + ".cbf_retreat_h_enter") {
        params_.cbf_retreat_h_enter = v;
      } else if (name == plugin_name_ + ".cbf_retreat_h_exit") {
        params_.cbf_retreat_h_exit = v;
      } else if (name == plugin_name_ + ".cbf_retreat_speed") {
        params_.cbf_retreat_speed = v;
      } else if (name == plugin_name_ + ".cbf_retreat_rotation_speed") {
        params_.cbf_retreat_rotation_speed = v;
      } else if (name == plugin_name_ + ".cbf_retreat_lookahead_s") {
        params_.cbf_retreat_lookahead_s = v;
      }
    } else if (type == rclcpp::ParameterType::PARAMETER_BOOL) {
      const bool v = p.as_bool();
      if (name == plugin_name_ + ".log_enabled") {
        params_.log_enabled = v;
      } else if (name == plugin_name_ + ".cbf_retreat_enabled") {
        params_.cbf_retreat_enabled = v;
      }
    } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
      const int v = static_cast<int>(p.as_int());
      if (name == plugin_name_ + ".log_lidar_every_n_ticks") {
        params_.log_lidar_every_n_ticks = v;
      } else if (name == plugin_name_ + ".cbf_n_predict_steps") {
        params_.cbf_n_predict_steps = v;
      }
    }
  }
  return result;
}

}  // namespace nav2_constrained_controller
