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

#include <cmath>
#include <exception>

#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

SourceBase::SourceBase()
: tf_buffer_(nullptr), source_type_(SOURCE_BASE), source_name_(""),
  source_topic_(""), source_frame_id_(""), base_frame_id_(""),
  transform_tolerance_(0.0), max_time_shift_(0.0)
{
}

SourceBase::SourceBase(
  const nav2_util::LifecycleNode::WeakPtr & node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string source_name,
  const std::string base_frame_id,
  const double transform_tolerance,
  const double max_time_shift)
: node_(node), tf_buffer_(tf_buffer), source_type_(SOURCE_BASE), source_name_(source_name),
  source_topic_(""), source_frame_id_(""), base_frame_id_(base_frame_id),
  transform_tolerance_(transform_tolerance), max_time_shift_(max_time_shift)
{
  auto node_sptr = node_.lock();
  if (node_sptr) {
    RCLCPP_INFO(node_sptr->get_logger(), "Creating SourceBase");
  }
}

SourceBase::~SourceBase()
{
  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO(node->get_logger(), "Destroying SourceBase");
  }

  tf_buffer_.reset();
}

bool SourceBase::getParameters()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  try {
    nav2_util::declare_parameter_if_not_declared(
      node, source_name_ + ".topic",
      rclcpp::ParameterValue("scan"));  // Set deafult topic for laser scanner
    source_topic_ = node->get_parameter(source_name_ + ".topic").as_string();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      node->get_logger(), "Error while getting basic source parameters: %s", ex.what());
    return false;
  }

  return true;
}

SourceType SourceBase::getSourceType()
{
  return source_type_;
}

std::string SourceBase::getSourceTypeStr()
{
  if (source_type_ == SOURCE_BASE) {
    return "base";
  } else if (source_type_ == SCAN) {
    return "scan";
  } else {  // source_type_ == POINTCLOUD
    return "pointcloud";
  }
}

bool SourceBase::getTransform(
  const std::string to_frame,
  const std::string from_frame,
  tf2::Transform & tf2_transform)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!tf_buffer_) {
    RCLCPP_ERROR(node->get_logger(), "tf_buffer_ is not initialized");
    return false;
  }

  geometry_msgs::msg::TransformStamped transform;
  tf2_transform.setIdentity();  // initialize by identical transform

  if (from_frame == to_frame) {
    // tf2_transform is already identical. Do nothing.
    return true;
  }

  try {
    const tf2::Duration tr_tol = tf2::durationFromSec(transform_tolerance_);
    // Obtaining the transform to get data from one to another frame
    transform = tf_buffer_->lookupTransform(
      to_frame, from_frame, tf2::TimePointZero, tr_tol);
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to get \"%s\"->\"%s\" frame transform: %s",
      source_frame_id_.c_str(), base_frame_id_.c_str(), e.what());
    return false;
  }

  // Convert TransformStamped to TF2 transform
  tf2::fromMsg(transform.transform, tf2_transform);
  return true;
}

void SourceBase::getData(
  std::vector<Point> & data, const rclcpp::Time & curr_time, const Velocity & velocity)
{
  std::lock_guard<mutex_t> lock(data_mutex_);

  fixData(curr_time, velocity);
  data.insert(data.end(), data_fixed_.data.begin(), data_fixed_.data.end());
}

void SourceBase::fixData(const rclcpp::Time & curr_time, const Velocity & velocity)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Start opearting with data
  std::lock_guard<mutex_t> lock(data_mutex_);
  data_fixed_.data.clear();
  data_fixed_.time = data_.time;

  // Time shift between current time and time of latest data received.
  // Used in movement correction of data
  double dt;
  try {
    dt = (curr_time - data_.time).seconds();
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      node->get_logger(),
      "Can not get difference between collision monitor and %s source time: %s",
      source_name_.c_str(), ex.what());
    dt = 0.0;
  }
  // Warn and ignore the source data if it and Collision Monitor node have different time stamps
  if (std::fabs(dt) > max_time_shift_) {
    RCLCPP_WARN(
      node->get_logger(),
      "%s source and collision monitor node timestamps differ on %f seconds. Ignoring the source.",
      source_name_.c_str(), dt);
    return;
  }

  for (Point p : data_.data) {
    // Fix each point on dt * velocity
    transformPoint(velocity, dt, p);

    // Fill data_fixed_.data array
    data_fixed_.data.push_back(p);
  }
}

}  // namespace nav2_collision_monitor
