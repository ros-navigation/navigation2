// Copyright (c) 2022 Samsung Research
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

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_velocity_smoother/velocity_smoother.hpp"

using namespace std::chrono_literals;
using nav2::declare_parameter_if_not_declared;
using std::placeholders::_1;
using rcl_interfaces::msg::ParameterType;

namespace nav2_velocity_smoother
{

VelocitySmoother::VelocitySmoother(const rclcpp::NodeOptions & options)
: LifecycleNode("velocity_smoother", "", options),
  last_command_time_{0, 0, get_clock()->get_clock_type()}
{
}

VelocitySmoother::~VelocitySmoother()
{
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
}

nav2::CallbackReturn
VelocitySmoother::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring velocity smoother");
  auto node = shared_from_this();
  std::string feedback_type;
  double velocity_timeout_dbl;

  // Smoothing metadata
  declare_parameter_if_not_declared(node, "smoothing_frequency", rclcpp::ParameterValue(20.0));
  declare_parameter_if_not_declared(
    node, "feedback", rclcpp::ParameterValue(std::string("OPEN_LOOP")));
  declare_parameter_if_not_declared(node, "scale_velocities", rclcpp::ParameterValue(false));
  node->get_parameter("smoothing_frequency", smoothing_frequency_);
  node->get_parameter("feedback", feedback_type);
  node->get_parameter("scale_velocities", scale_velocities_);

  // Kinematics
  declare_parameter_if_not_declared(
    node, "max_velocity", rclcpp::ParameterValue(std::vector<double>{0.50, 0.0, 2.5}));
  declare_parameter_if_not_declared(
    node, "min_velocity", rclcpp::ParameterValue(std::vector<double>{-0.50, 0.0, -2.5}));
  declare_parameter_if_not_declared(
    node, "max_accel", rclcpp::ParameterValue(std::vector<double>{2.5, 0.0, 3.2}));
  declare_parameter_if_not_declared(
    node, "max_decel", rclcpp::ParameterValue(std::vector<double>{-2.5, 0.0, -3.2}));
  node->get_parameter("max_velocity", max_velocities_);
  node->get_parameter("min_velocity", min_velocities_);
  node->get_parameter("max_accel", max_accels_);
  node->get_parameter("max_decel", max_decels_);

  // Get feature parameters
  declare_parameter_if_not_declared(node, "odom_topic", rclcpp::ParameterValue("odom"));
  declare_parameter_if_not_declared(node, "odom_duration", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, "deadband_velocity", rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0}));
  declare_parameter_if_not_declared(node, "velocity_timeout", rclcpp::ParameterValue(1.0));
  node->get_parameter("odom_topic", odom_topic_);
  node->get_parameter("odom_duration", odom_duration_);
  node->get_parameter("deadband_velocity", deadband_velocities_);
  node->get_parameter("velocity_timeout", velocity_timeout_dbl);
  velocity_timeout_ = rclcpp::Duration::from_seconds(velocity_timeout_dbl);

  // Check if parameters are properly set
  size_t size = max_velocities_.size();
  is_6dof_ = (size == 6);

  if ((size != 3 && size != 6) ||
    min_velocities_.size() != size ||
    max_accels_.size() != size ||
    max_decels_.size() != size ||
    deadband_velocities_.size() != size)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid setting of kinematic and/or deadband limits!"
      " All limits must be size of 3 (x, y, theta) or 6 (x, y, z, r, p, y)");
    on_cleanup(state);
    return nav2::CallbackReturn::FAILURE;
  }

  for (unsigned int i = 0; i != size; i++) {
    if (max_decels_[i] > 0.0) {
      RCLCPP_ERROR(
        get_logger(),
        "Positive values set of deceleration! These should be negative to slow down!");
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
    if (max_accels_[i] < 0.0) {
      RCLCPP_ERROR(
        get_logger(),
        "Negative values set of acceleration! These should be positive to speed up!");
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
    if (min_velocities_[i] > 0.0) {
      RCLCPP_ERROR(
        get_logger(), "Positive values set of min_velocities! These should be negative!");
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
    if (max_velocities_[i] < 0.0) {
      RCLCPP_ERROR(
        get_logger(), "Negative values set of max_velocities! These should be positive!");
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
    if (min_velocities_[i] > max_velocities_[i]) {
      RCLCPP_ERROR(get_logger(), "Min velocities are higher than max velocities!");
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
  }

  // Get control type
  if (feedback_type == "OPEN_LOOP") {
    open_loop_ = true;
  } else if (feedback_type == "CLOSED_LOOP") {
    open_loop_ = false;
    odom_smoother_ = std::make_unique<nav2_util::OdomSmoother>(node, odom_duration_, odom_topic_);
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid feedback_type, options are OPEN_LOOP and CLOSED_LOOP.");
    on_cleanup(state);
    return nav2::CallbackReturn::FAILURE;
  }

  // Setup inputs / outputs
  smoothed_cmd_pub_ = std::make_unique<nav2_util::TwistPublisher>(node, "cmd_vel_smoothed");
  cmd_sub_ = std::make_unique<nav2_util::TwistSubscriber>(
    node,
    "cmd_vel",
    std::bind(&VelocitySmoother::inputCommandCallback, this, std::placeholders::_1),
    std::bind(&VelocitySmoother::inputCommandStampedCallback, this, std::placeholders::_1));

  declare_parameter_if_not_declared(node, "use_realtime_priority", rclcpp::ParameterValue(false));
  bool use_realtime_priority = false;
  node->get_parameter("use_realtime_priority", use_realtime_priority);
  if (use_realtime_priority) {
    try {
      nav2::setSoftRealTimePriority();
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
  }

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
VelocitySmoother::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");
  smoothed_cmd_pub_->on_activate();
  double timer_duration_ms = 1000.0 / smoothing_frequency_;
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(timer_duration_ms)),
    std::bind(&VelocitySmoother::smootherTimer, this));

  dyn_params_handler_ = this->add_on_set_parameters_callback(
    std::bind(&VelocitySmoother::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();
  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
VelocitySmoother::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  smoothed_cmd_pub_->on_deactivate();

  remove_on_set_parameters_callback(dyn_params_handler_.get());
  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();
  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
VelocitySmoother::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  smoothed_cmd_pub_.reset();
  odom_smoother_.reset();
  cmd_sub_.reset();
  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
VelocitySmoother::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2::CallbackReturn::SUCCESS;
}

void VelocitySmoother::inputCommandStampedCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  // If message contains NaN or Inf, ignore
  if (!nav2_util::validateTwist(msg->twist)) {
    RCLCPP_ERROR(get_logger(), "Velocity message contains NaNs or Infs! Ignoring as invalid!");
    return;
  }

  command_ = msg;
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
    last_command_time_ = now();
  } else {
    last_command_time_ = msg->header.stamp;
  }
}

void VelocitySmoother::inputCommandCallback(
  geometry_msgs::msg::Twist::SharedPtr msg)
{
  auto twist_stamped = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_stamped->twist = *msg;
  inputCommandStampedCallback(twist_stamped);
}

double VelocitySmoother::findEtaConstraint(
  const double v_curr, const double v_cmd, const double accel, const double decel)
{
  // Exploiting vector scaling properties
  double dv = v_cmd - v_curr;

  double v_component_max;
  double v_component_min;

  // Accelerating if magnitude of v_cmd is above magnitude of v_curr
  // and if v_cmd and v_curr have the same sign (i.e. speed is NOT passing through 0.0)
  // Decelerating otherwise
  if (abs(v_cmd) >= abs(v_curr) && v_curr * v_cmd >= 0.0) {
    v_component_max = accel / smoothing_frequency_;
    v_component_min = -accel / smoothing_frequency_;
  } else {
    v_component_max = -decel / smoothing_frequency_;
    v_component_min = decel / smoothing_frequency_;
  }

  if (v_cmd == 0.0) {
    return -1.0;
  }

  if (dv > v_component_max) {
    return ( v_component_max + v_curr ) / v_cmd;
  }

  if (dv < v_component_min) {
    return ( v_component_min + v_curr ) / v_cmd;
  }

  return -1.0;
}

double VelocitySmoother::applyConstraints(
  const double v_curr, const double v_cmd,
  const double accel, const double decel, const double eta)
{
  double v_scaled = eta * v_cmd;
  double v_diff = v_scaled - v_curr;

  double v_component_max;
  double v_component_min;

  // Accelerating if magnitude of v_cmd is above magnitude of v_curr
  // and if v_cmd and v_curr have the same sign (i.e. speed is NOT passing through 0.0)
  // Decelerating otherwise
  if (abs(v_cmd) >= abs(v_curr) && v_curr * v_cmd >= 0.0) {
    v_component_max = accel / smoothing_frequency_;
    v_component_min = -accel / smoothing_frequency_;
  } else {
    v_component_max = -decel / smoothing_frequency_;
    v_component_min = decel / smoothing_frequency_;
  }

  auto v_diff_clamped = std::clamp(v_diff, v_component_min, v_component_max);
  auto v_cmd_restricted = v_curr + v_diff_clamped;

  if(std::abs(v_scaled - v_cmd_restricted) > 0.0001){
    RCLCPP_DEBUG(get_logger(), "Clamped velocity change: eta * v_cmd = %f, clamped to %f", v_scaled, v_cmd_restricted);
  }

  return v_cmd_restricted;
}

void VelocitySmoother::smootherTimer()
{
  // Wait until the first command is received
  if (!command_) {
    return;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header = command_->header;

  // Check for velocity timeout. If nothing received, publish zeros to apply deceleration
  if (now() - last_command_time_ > velocity_timeout_) {
    if (last_cmd_.twist == geometry_msgs::msg::Twist() || stopped_) {
      stopped_ = true;
      return;
    }
    *command_ = geometry_msgs::msg::TwistStamped();
    command_->header.stamp = now();
  }

  stopped_ = false;

  // Get current velocity based on feedback type
  geometry_msgs::msg::TwistStamped current_;
  if (open_loop_) {
    current_ = last_cmd_;
  } else {
    current_ = odom_smoother_->getTwistStamped();
  }

  RCLCPP_DEBUG(get_logger(), "Current (odom) velocity: [%f, %f, %f]", current_.twist.linear.x, current_.twist.linear.y, current_.twist.angular.z);
  RCLCPP_DEBUG(get_logger(), "Input velocity: [%f, %f, %f]", command_->twist.linear.x, command_->twist.linear.y, command_->twist.angular.z);

  // Apply absolute velocity restrictions to the command
  if(!is_6dof_) {
    command_->twist.linear.x = std::clamp(
      command_->twist.linear.x, min_velocities_[0],
      max_velocities_[0]);
    command_->twist.linear.y = std::clamp(
      command_->twist.linear.y, min_velocities_[1],
      max_velocities_[1]);
    command_->twist.angular.z = std::clamp(
      command_->twist.angular.z, min_velocities_[2],
      max_velocities_[2]);
  } else {
    command_->twist.linear.x = std::clamp(
      command_->twist.linear.x, min_velocities_[0],
      max_velocities_[0]);
    command_->twist.linear.y = std::clamp(
      command_->twist.linear.y, min_velocities_[1],
      max_velocities_[1]);
    command_->twist.linear.z = std::clamp(
      command_->twist.linear.z, min_velocities_[2],
      max_velocities_[2]);
    command_->twist.angular.x = std::clamp(
      command_->twist.angular.x, min_velocities_[3],
      max_velocities_[3]);
    command_->twist.angular.y = std::clamp(
      command_->twist.angular.y, min_velocities_[4],
      max_velocities_[4]);
    command_->twist.angular.z = std::clamp(
      command_->twist.angular.z, min_velocities_[5],
      max_velocities_[5]);
  }

  RCLCPP_DEBUG(get_logger(), "Clamped velocity: [%f, %f, %f]", command_->twist.linear.x, command_->twist.linear.y, command_->twist.angular.z);

  // Find if any component is not within the acceleration constraints. If so, store the most
  // significant scale factor to apply to the vector <dvx, dvy, dvw>, eta, to reduce all axes
  // proportionally to follow the same direction, within change of velocity bounds.
  // In case eta reduces another axis out of its own limit, apply accel constraint to guarantee
  // output is within limits, even if it deviates from requested command slightly.
  double eta = 1.0;
  if (scale_velocities_) {
    double curr_eta = -1.0;
    if(!is_6dof_) {
      curr_eta = findEtaConstraint(
        current_.twist.linear.x, command_->twist.linear.x, max_accels_[0], max_decels_[0]);
      if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
        eta = curr_eta;
      }

      curr_eta = findEtaConstraint(
        current_.twist.linear.y, command_->twist.linear.y, max_accels_[1], max_decels_[1]);
      if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
        eta = curr_eta;
      }

      curr_eta = findEtaConstraint(
        current_.twist.angular.z, command_->twist.angular.z, max_accels_[2], max_decels_[2]);
      if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
        eta = curr_eta;
      }
    } else {
      curr_eta = findEtaConstraint(
        current_.twist.linear.x, command_->twist.linear.x, max_accels_[0], max_decels_[0]);
      if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
        eta = curr_eta;
      }

      curr_eta = findEtaConstraint(
        current_.twist.linear.y, command_->twist.linear.y, max_accels_[1], max_decels_[1]);
      if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
        eta = curr_eta;
      }

      curr_eta = findEtaConstraint(
        current_.twist.linear.z, command_->twist.linear.z, max_accels_[2], max_decels_[2]);
      if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
        eta = curr_eta;
      }

      curr_eta = findEtaConstraint(
        current_.twist.angular.x, command_->twist.angular.x, max_accels_[3], max_decels_[3]);
      if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
        eta = curr_eta;
      }

      curr_eta = findEtaConstraint(
        current_.twist.angular.y, command_->twist.angular.y, max_accels_[4], max_decels_[4]);
      if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
        eta = curr_eta;
      }

      curr_eta = findEtaConstraint(
        current_.twist.angular.z, command_->twist.angular.z, max_accels_[5], max_decels_[5]);
      if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
        eta = curr_eta;
      }
    }
  }
  RCLCPP_DEBUG(get_logger(), "Eta = %f", eta);

  if(!is_6dof_) {
    cmd_vel->twist.linear.x = applyConstraints(
      current_.twist.linear.x, command_->twist.linear.x, max_accels_[0], max_decels_[0], eta);
    cmd_vel->twist.linear.y = applyConstraints(
      current_.twist.linear.y, command_->twist.linear.y, max_accels_[1], max_decels_[1], eta);
    cmd_vel->twist.angular.z = applyConstraints(
      current_.twist.angular.z, command_->twist.angular.z, max_accels_[2], max_decels_[2], eta);
  } else {
    cmd_vel->twist.linear.x = applyConstraints(
      current_.twist.linear.x, command_->twist.linear.x, max_accels_[0], max_decels_[0], eta);
    cmd_vel->twist.linear.y = applyConstraints(
      current_.twist.linear.y, command_->twist.linear.y, max_accels_[1], max_decels_[1], eta);
    cmd_vel->twist.linear.z = applyConstraints(
      current_.twist.linear.z, command_->twist.linear.z, max_accels_[2], max_decels_[2], eta);
    cmd_vel->twist.angular.x = applyConstraints(
      current_.twist.angular.x, command_->twist.angular.x, max_accels_[3], max_decels_[3], eta);
    cmd_vel->twist.angular.y = applyConstraints(
      current_.twist.angular.y, command_->twist.angular.y, max_accels_[4], max_decels_[4], eta);
    cmd_vel->twist.angular.z = applyConstraints(
      current_.twist.angular.z, command_->twist.angular.z, max_accels_[5], max_decels_[5], eta);
  }

  last_cmd_ = *cmd_vel;


  // Apply deadband restrictions & publish
  if(!is_6dof_) {
    cmd_vel->twist.linear.x =
      fabs(cmd_vel->twist.linear.x) < deadband_velocities_[0] ? 0.0 : cmd_vel->twist.linear.x;
    cmd_vel->twist.linear.y =
      fabs(cmd_vel->twist.linear.y) < deadband_velocities_[1] ? 0.0 : cmd_vel->twist.linear.y;
    cmd_vel->twist.linear.z = command_->twist.linear.z;
    cmd_vel->twist.angular.x = command_->twist.angular.x;
    cmd_vel->twist.angular.y = command_->twist.angular.y;
    cmd_vel->twist.angular.z =
      fabs(cmd_vel->twist.angular.z) < deadband_velocities_[2] ? 0.0 : cmd_vel->twist.angular.z;
  } else {
    cmd_vel->twist.linear.x =
      fabs(cmd_vel->twist.linear.x) < deadband_velocities_[0] ? 0.0 : cmd_vel->twist.linear.x;
    cmd_vel->twist.linear.y =
      fabs(cmd_vel->twist.linear.y) < deadband_velocities_[1] ? 0.0 : cmd_vel->twist.linear.y;
    cmd_vel->twist.linear.z =
      fabs(cmd_vel->twist.linear.z) < deadband_velocities_[2] ? 0.0 : cmd_vel->twist.linear.z;
    cmd_vel->twist.angular.x =
      fabs(cmd_vel->twist.angular.x) < deadband_velocities_[3] ? 0.0 : cmd_vel->twist.angular.x;
    cmd_vel->twist.angular.y =
      fabs(cmd_vel->twist.angular.y) < deadband_velocities_[4] ? 0.0 : cmd_vel->twist.angular.y;
    cmd_vel->twist.angular.z =
      fabs(cmd_vel->twist.angular.z) < deadband_velocities_[5] ? 0.0 : cmd_vel->twist.angular.z;
  }
  smoothed_cmd_pub_->publish(std::move(cmd_vel));
}

rcl_interfaces::msg::SetParametersResult
VelocitySmoother::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find('.') != std::string::npos) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == "smoothing_frequency") {
        smoothing_frequency_ = parameter.as_double();
        if (timer_) {
          timer_->cancel();
          timer_.reset();
        }

        double timer_duration_ms = 1000.0 / smoothing_frequency_;
        timer_ = this->create_wall_timer(
          std::chrono::milliseconds(static_cast<int>(timer_duration_ms)),
          std::bind(&VelocitySmoother::smootherTimer, this));
      } else if (param_name == "velocity_timeout") {
        velocity_timeout_ = rclcpp::Duration::from_seconds(parameter.as_double());
      } else if (param_name == "odom_duration") {
        odom_duration_ = parameter.as_double();
        odom_smoother_ =
          std::make_unique<nav2_util::OdomSmoother>(
          shared_from_this(), odom_duration_, odom_topic_);
      }
    } else if (param_type == ParameterType::PARAMETER_DOUBLE_ARRAY) {
      size_t size = is_6dof_ ? 6 : 3;
      if (parameter.as_double_array().size() != size) {
        RCLCPP_WARN(get_logger(), "Invalid size of parameter %s. Must be size %ld",
            param_name.c_str(), size);
        result.successful = false;
        break;
      }

      if (param_name == "max_velocity") {
        for (unsigned int i = 0; i != size; i++) {
          if (parameter.as_double_array()[i] < 0.0) {
            RCLCPP_WARN(
              get_logger(),
              "Negative values set of max_velocity! These should be positive!");
            result.successful = false;
          }
        }
        if (result.successful) {
          max_velocities_ = parameter.as_double_array();
        }
      } else if (param_name == "min_velocity") {
        for (unsigned int i = 0; i != size; i++) {
          if (parameter.as_double_array()[i] > 0.0) {
            RCLCPP_WARN(
              get_logger(),
              "Positive values set of min_velocity! These should be negative!");
            result.successful = false;
          }
        }
        if (result.successful) {
          min_velocities_ = parameter.as_double_array();
        }
      } else if (param_name == "max_accel") {
        for (unsigned int i = 0; i != size; i++) {
          if (parameter.as_double_array()[i] < 0.0) {
            RCLCPP_WARN(
              get_logger(),
              "Negative values set of acceleration! These should be positive to speed up!");
            result.successful = false;
          }
        }
        if (result.successful) {
          max_accels_ = parameter.as_double_array();
        }
      } else if (param_name == "max_decel") {
        for (unsigned int i = 0; i != size; i++) {
          if (parameter.as_double_array()[i] > 0.0) {
            RCLCPP_WARN(
              get_logger(),
              "Positive values set of deceleration! These should be negative to slow down!");
            result.successful = false;
          }
        }
        if (result.successful) {
          max_decels_ = parameter.as_double_array();
        }
      } else if (param_name == "deadband_velocity") {
        deadband_velocities_ = parameter.as_double_array();
      }
    } else if (param_type == ParameterType::PARAMETER_STRING) {
      if (param_name == "feedback") {
        if (parameter.as_string() == "OPEN_LOOP") {
          open_loop_ = true;
          odom_smoother_.reset();
        } else if (parameter.as_string() == "CLOSED_LOOP") {
          open_loop_ = false;
          odom_smoother_ =
            std::make_unique<nav2_util::OdomSmoother>(
            shared_from_this(), odom_duration_, odom_topic_);
        } else {
          RCLCPP_WARN(
            get_logger(), "Invalid feedback_type, options are OPEN_LOOP and CLOSED_LOOP.");
          result.successful = false;
          break;
        }
      } else if (param_name == "odom_topic") {
        odom_topic_ = parameter.as_string();
        odom_smoother_ =
          std::make_unique<nav2_util::OdomSmoother>(
          shared_from_this(), odom_duration_, odom_topic_);
      }
    }
  }

  return result;
}

}  // namespace nav2_velocity_smoother

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_velocity_smoother::VelocitySmoother)
