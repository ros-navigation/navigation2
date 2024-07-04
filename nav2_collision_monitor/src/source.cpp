// Copyright (c) 2022 Samsung R&D Institute Russia
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

#include "nav2_collision_monitor/source.hpp"

#include <exception>

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

Source::Source(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
: node_(node), source_name_(source_name), tf_buffer_(tf_buffer),
  base_frame_id_(base_frame_id), global_frame_id_(global_frame_id),
  transform_tolerance_(transform_tolerance), source_timeout_(source_timeout),
  base_shift_correction_(base_shift_correction)
{
}

Source::~Source()
{
}

bool Source::configure()
{
  auto node = node_.lock();

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&Source::dynamicParametersCallback, this, std::placeholders::_1));

  return true;
}

void Source::getCommonParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".topic",
    rclcpp::ParameterValue("scan"));  // Set deafult topic for laser scanner
  source_topic = node->get_parameter(source_name_ + ".topic").as_string();

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".enabled", rclcpp::ParameterValue(true));
  enabled_ = node->get_parameter(source_name_ + ".enabled").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".source_timeout",
    rclcpp::ParameterValue(source_timeout_.seconds()));      // node source_timeout by default
  source_timeout_ = rclcpp::Duration::from_seconds(
    node->get_parameter(source_name_ + ".source_timeout").as_double());
}

bool Source::sourceValid(
  const rclcpp::Time & source_time,
  const rclcpp::Time & curr_time) const
{
  // Source is considered as not valid, if latest received data timestamp is earlier
  // than current time by source_timeout_ interval
  const rclcpp::Duration dt = curr_time - source_time;
  if (source_timeout_.seconds() != 0.0 && dt > source_timeout_) {
    RCLCPP_WARN(
      logger_,
      "[%s]: Latest source and current collision monitor node timestamps differ on %f seconds. "
      "Ignoring the source.",
      source_name_.c_str(), dt.seconds());
    return false;
  }

  return true;
}

bool Source::getEnabled() const
{
  return enabled_;
}

std::string Source::getSourceName() const
{
  return source_name_;
}

rclcpp::Duration Source::getSourceTimeout() const
{
  return source_timeout_;
}

rcl_interfaces::msg::SetParametersResult
Source::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      if (param_name == source_name_ + "." + "enabled") {
        enabled_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_collision_monitor
