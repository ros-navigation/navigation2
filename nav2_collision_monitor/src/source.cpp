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
#include <iterator>

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "nav2_ros_common/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_ros_common/tf2_factories.hpp"

namespace nav2_collision_monitor
{

Source::Source(
  const nav2::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const nav2::TransformBuffer::SharedPtr tf_buffer,
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
  auto node = node_.lock();
  if (post_set_params_handler_ && node) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
  }
  post_set_params_handler_.reset();
  if (on_set_params_handler_ && node) {
    node->remove_on_set_parameters_callback(on_set_params_handler_.get());
  }
  on_set_params_handler_.reset();
}

bool Source::configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Configure the exclusion zones (if any) declared for this source
  const std::vector<std::string> zone_names =
    node->declare_or_get_parameter<std::vector<std::string>>(
    source_name_ + ".exclusion_zones", std::vector<std::string>());

  for (const std::string & zone_name : zone_names) {
    auto zone = std::make_shared<ExclusionZone>(
      node, zone_name, tf_buffer_, base_frame_id_, global_frame_id_,
      transform_tolerance_, base_shift_correction_);
    if (!zone->configure()) {
      RCLCPP_ERROR(
        logger_, "[%s]: Failed to configure exclusion zone '%s'",
        source_name_.c_str(), zone_name.c_str());
      return false;
    }
    exclusion_zones_.push_back(zone);
  }

  // Add callback for dynamic parameters
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &Source::updateParametersCallback,
      this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &Source::validateParameterUpdatesCallback,
      this, std::placeholders::_1));

  return true;
}

bool Source::getData(
  const rclcpp::Time & curr_time,
  std::vector<Point> & data)
{
  // Collect this source's points into a private buffer so exclusion-zone
  // masking only ever considers data produced by this source.
  std::vector<Point> source_data;
  if (!getSourceData(curr_time, source_data)) {
    return false;
  }

  // Mask out points that fall inside any enabled exclusion zone.
  if (!exclusion_zones_.empty() && !source_data.empty()) {
    for (const auto & zone : exclusion_zones_) {
      zone->apply(curr_time, source_data);
    }
  }

  // Append the surviving points to the caller's array.
  data.insert(
    data.end(),
    std::make_move_iterator(source_data.begin()),
    std::make_move_iterator(source_data.end()));

  return true;
}

void Source::activate()
{
  for (const auto & zone : exclusion_zones_) {
    zone->activate();
  }
}

void Source::deactivate()
{
  for (const auto & zone : exclusion_zones_) {
    zone->deactivate();
  }
}

void Source::publishExclusionZones() const
{
  for (const auto & zone : exclusion_zones_) {
    zone->publish();
  }
}

void Source::getCommonParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  source_topic = node->declare_or_get_parameter(
    source_name_ + ".topic", std::string("scan"));  // Set default topic for laser scanner

  enabled_ = node->declare_or_get_parameter(
    source_name_ + ".enabled", true);

  source_timeout_ = rclcpp::Duration::from_seconds(
    node->declare_or_get_parameter(
      source_name_ + ".source_timeout",
      source_timeout_.seconds()));  // node source_timeout by default
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
  std::lock_guard<std::mutex> lock_reinit(mutex_);
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

rcl_interfaces::msg::SetParametersResult Source::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & /*parameters*/)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void Source::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(source_name_ + ".") != 0) {
      continue;
    }
    if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      if (param_name == source_name_ + "." + "enabled") {
        enabled_ = parameter.as_bool();
      }
    }
  }
}

bool Source::getTransform(
  const rclcpp::Time & curr_time,
  const std_msgs::msg::Header & data_header,
  tf2::Transform & tf_transform) const
{
  if (base_shift_correction_) {
    if (
      !nav2_util::getTransform(
        data_header.frame_id, data_header.stamp,
        base_frame_id_, curr_time, global_frame_id_,
        transform_tolerance_, tf_buffer_, tf_transform))
    {
      return false;
    }
  } else {
    if (
      !nav2_util::getTransform(
        data_header.frame_id, base_frame_id_,
        transform_tolerance_, tf_buffer_, tf_transform))
    {
      return false;
    }
  }
  return true;
}

}  // namespace nav2_collision_monitor
