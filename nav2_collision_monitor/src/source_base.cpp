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

#include "nav2_collision_monitor/source_base.hpp"
#include "nav2_collision_monitor/kinematics.hpp"

#include <exception>

#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

SourceBase::SourceBase(
  const nav2_util::LifecycleNode::WeakPtr & node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & source_name,
  const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance,
  const tf2::Duration & data_timeout)
: node_(node), tf_buffer_(tf_buffer), source_name_(source_name),
  source_frame_id_(""), base_frame_id_(base_frame_id), fixed_frame_id_(""),
  transform_tolerance_(transform_tolerance), data_timeout_(data_timeout)
{
  RCLCPP_INFO(logger_, "[%s]: Creating SourceBase", source_name_.c_str());

  auto node_shared = node_.lock();
  if (!node_shared) {
    throw std::runtime_error{"Failed to lock node"};
  }

  data_stamp_ = {0, 0, node_shared->get_clock()->get_clock_type()};
}

SourceBase::~SourceBase()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying SourceBase", source_name_.c_str());

  tf_buffer_.reset();
}

void SourceBase::setFixedFrameId(const std::string & fixed_frame_id)
{
  fixed_frame_id_ = fixed_frame_id;
}

void SourceBase::getParameters(std::string & source_topic)
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

bool SourceBase::sourceValid(const rclcpp::Time & curr_time)
{
  // Check that latest received data timestamp is earlier
  // than current time more than data_timeout_ interval
  const rclcpp::Duration dt = curr_time - data_stamp_;
  if (dt > data_timeout_) {
    RCLCPP_WARN(
      logger_,
      "[%s]: Latest source and current collision monitor node timestamps differ on %f seconds. "
      "Ignoring the source.",
      source_name_.c_str(), dt.seconds());
    return false;
  }

  return true;
}

bool SourceBase::getSourceBaseTransform(
  const rclcpp::Time & curr_time,
  const rclcpp::Time & source_time,
  tf2::Transform & tf2_transform)
{
  geometry_msgs::msg::TransformStamped transform;
  tf2_transform.setIdentity();  // initialize by identical transform

  try {
    // Obtaining the transform to get data from source_frame_id_ to base_frame_id_ frame.
    // This also considers the time shift between the moment when the source data was obtained and current time.
    transform = tf_buffer_->lookupTransform(
      base_frame_id_, curr_time,
      source_frame_id_, source_time,
      fixed_frame_id_, transform_tolerance_);
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Failed to get \"%s\"->\"%s\" frame transform: %s",
      source_name_.c_str(), source_frame_id_.c_str(), base_frame_id_.c_str(), e.what());
    return false;
  }

  // Convert TransformStamped to TF2 transform
  tf2::fromMsg(transform.transform, tf2_transform);
  return true;
}

}  // namespace nav2_collision_monitor
