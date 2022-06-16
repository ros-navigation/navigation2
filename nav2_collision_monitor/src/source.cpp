// Copyright (c) 2022 Samsung Research Russia
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

#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

Source::Source(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name)
: node_(node), source_name_(source_name)
{
}

Source::~Source()
{
}

void Source::getBasicParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".topic",
    rclcpp::ParameterValue("scan"));  // Set deafult topic for laser scanner
  source_topic = node->get_parameter(source_name_ + ".topic").as_string();
}

bool Source::sourceValid(
  const rclcpp::Time & source_time,
  const rclcpp::Time & curr_time,
  const rclcpp::Duration & data_timeout) const
{
  // Source is considered as not valid, if latest received data timestamp is earlier
  // than current time by data_timeout interval
  const rclcpp::Duration dt = curr_time - source_time;
  if (dt > data_timeout) {
    RCLCPP_WARN(
      logger_,
      "[%s]: Latest source and current collision monitor node timestamps differ on %f seconds. "
      "Ignoring the source.",
      source_name_.c_str(), dt.seconds());
    return false;
  }

  return true;
}

bool Source::getTransform(
  const std::string & source_frame_id,
  const rclcpp::Time & source_time,
  const std::string & target_frame_id,
  const rclcpp::Time & target_time,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  tf2::Transform & tf2_transform) const
{
  geometry_msgs::msg::TransformStamped transform;
  tf2_transform.setIdentity();  // initialize by identical transform

  try {
    // Obtaining the transform to get data from source to target frame.
    // This also considers the time shift between source and target.
    transform = tf_buffer->lookupTransform(
      target_frame_id, target_time,
      source_frame_id, source_time,
      global_frame_id, transform_tolerance);
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Failed to get \"%s\"->\"%s\" frame transform: %s",
      source_name_.c_str(), source_frame_id.c_str(), target_frame_id.c_str(), e.what());
    return false;
  }

  // Convert TransformStamped to TF2 transform
  tf2::fromMsg(transform.transform, tf2_transform);
  return true;
}

}  // namespace nav2_collision_monitor
