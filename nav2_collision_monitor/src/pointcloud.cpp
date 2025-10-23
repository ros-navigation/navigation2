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

#include "nav2_collision_monitor/pointcloud.hpp"

#include <functional>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/transform_datatypes.h"

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_collision_monitor
{

PointCloud::PointCloud(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
: Source(
    node, source_name, tf_buffer, base_frame_id, global_frame_id,
    transform_tolerance, source_timeout, base_shift_correction),
  data_(nullptr)
{
  RCLCPP_INFO(logger_, "[%s]: Creating PointCloud", source_name_.c_str());
}

PointCloud::~PointCloud()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying PointCloud", source_name_.c_str());
  data_sub_.reset();
}

void PointCloud::configure()
{
  Source::configure();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;

  getParameters(source_topic);

  rclcpp::QoS pointcloud_qos = rclcpp::SensorDataQoS();  // set to default
  data_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    source_topic, pointcloud_qos,
    std::bind(&PointCloud::dataCallback, this, std::placeholders::_1));
}

bool PointCloud::getData(
  const rclcpp::Time & curr_time,
  std::vector<Point> & data)
{
  // Ignore data from the source if it is not being published yet or
  // not published for a long time
  if (data_ == nullptr) {
    return false;
  }
  if (!sourceValid(data_->header.stamp, curr_time)) {
    return false;
  }

  tf2::Transform tf_transform;
  if (!getTransform(curr_time, data_->header, tf_transform)) {
    return false;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*data_, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*data_, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*data_, "z");

  bool height_present = false;
  for (const auto & field : data_->fields) {
    if (field.name == "height") {
      height_present = true;
    }
  }

// Reference height field
  std::string height_field{"z"};
  if (use_global_height_ && height_present) {
    height_field = "height";
  } else if (use_global_height_) {
    RCLCPP_ERROR(logger_, "[%s]: 'use_global_height' parameter true but height field not in cloud",
      source_name_.c_str());
    return false;
  }
  sensor_msgs::PointCloud2ConstIterator<float> iter_height(*data_, height_field);

  // Refill data array with PointCloud points in base frame
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    // Transform point coordinates from source frame -> to base frame
    tf2::Vector3 p_v3_s(*iter_x, *iter_y, *iter_z);

    double data_height = *iter_z;
    if (use_global_height_) {
      data_height = *iter_height;
      ++iter_height;
    }

    // Check range from sensor origin before transformation
    double range = p_v3_s.length();
    if (range < min_range_) {
      continue;
    }

    tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

    // Refill data array
    if (data_height >= min_height_ && data_height <= max_height_) {
      data.push_back({p_v3_b.x(), p_v3_b.y()});
    }
  }
  return true;
}

void PointCloud::getParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  getCommonParameters(source_topic);

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".min_height", rclcpp::ParameterValue(0.05));
  min_height_ = node->get_parameter(source_name_ + ".min_height").as_double();
  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".max_height", rclcpp::ParameterValue(0.5));
  max_height_ = node->get_parameter(source_name_ + ".max_height").as_double();
  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".min_range", rclcpp::ParameterValue(0.0));
  min_range_ = node->get_parameter(source_name_ + ".min_range").as_double();
  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".use_global_height", rclcpp::ParameterValue(false));
  use_global_height_ = node->get_parameter(source_name_ + ".use_global_height").as_bool();
}

void PointCloud::dataCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  data_ = msg;
}

}  // namespace nav2_collision_monitor
