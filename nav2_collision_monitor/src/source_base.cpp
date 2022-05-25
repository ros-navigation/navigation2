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
#include "nav2_collision_monitor/dynamics.hpp"

#include <cmath>
#include <exception>

#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_collision_monitor
{

SourceBase::SourceBase()
: node_(nullptr), tf_buffer_(nullptr), source_type_(SOURCE_BASE),
  source_topic_(""), source_frame_id_(""), base_frame_id_(""), transform_tolerance_(0.1)
{
}

SourceBase::SourceBase(
  nav2_util::LifecycleNode * node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string source_topic,
  const std::string base_frame_id,
  const double transform_tolerance,
  const double max_time_shift)
: node_(node), tf_buffer_(tf_buffer), source_type_(SOURCE_BASE),
  source_topic_(source_topic), source_frame_id_(""), base_frame_id_(base_frame_id),
  transform_tolerance_(transform_tolerance), max_time_shift_(max_time_shift)
{
  RCLCPP_INFO(node_->get_logger(), "Creating SourceBase");
}

SourceBase::~SourceBase()
{
  if (node_)
  {
    RCLCPP_INFO(node_->get_logger(), "Destroying SourceBase");
  }

  tf_buffer_.reset();
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
  } else {  // source_type_ == PCL
    return "PCL";
  }
}

bool SourceBase::getTransform(
  const std::string to_frame,
  const std::string from_frame,
  tf2::Transform & tf2_transform)
{
  if (!tf_buffer_) {
    RCLCPP_ERROR(node_->get_logger(), "tf_buffer_ is not initialized");
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
      node_->get_logger(),
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
      node_->get_logger(), "Can not get difference between current and source time: %s",
      ex.what());
    dt = 0.0;
  }
  // Warn if source and Collision Monitor node have different time stamps
  if (std::fabs(dt) > max_time_shift_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Source and collision monitor node time stamps differ more than %f seconds. Ignoring...",
      max_time_shift_);
    dt = 0.0;
  }

  for (Point p : data_.data) {
    // Move each point on dt * velocity
    fixPoint(velocity, dt, p);

    // Fill data_fixed_.data array
    data_fixed_.data.push_back(p);
  }
}

}  // namespace nav2_collision_monitor
