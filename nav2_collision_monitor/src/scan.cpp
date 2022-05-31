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

#include "nav2_collision_monitor/scan.hpp"

#include <cmath>

namespace nav2_collision_monitor
{

Scan::Scan(
  const nav2_util::LifecycleNode::WeakPtr & node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string source_name,
  const std::string base_frame_id,
  const double transform_tolerance,
  const double max_time_shift)
{
  node_ = node;
  auto node_sptr = node_.lock();
  if (node_sptr) {
    RCLCPP_INFO(node_sptr->get_logger(), "Creating Scan");
  }

  tf_buffer_ = tf_buffer;

  source_type_ = SCAN;
  source_name_ = source_name;
  base_frame_id_ = base_frame_id;

  transform_tolerance_ = transform_tolerance;
  max_time_shift_ = max_time_shift;
}

Scan::~Scan() {
  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO(node->get_logger(), "Destroying Scan");
  }

  tf_buffer_.reset();

  data_sub_.reset();
}

bool Scan::init()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Laser scanner has no its own parameters
  if (!SourceBase::getParameters()) {
    return false;
  }

  rclcpp::QoS scan_qos = rclcpp::SensorDataQoS();  // set to default
  data_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    source_topic_, scan_qos,
    std::bind(&Scan::dataCallback, this, std::placeholders::_1));

  return true;
}

void Scan::dataCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  std::lock_guard<mutex_t> lock(data_mutex_);

  // Obtaining source_frame_id_ from first message
  if (source_frame_id_.length() == 0) {
    source_frame_id_ = msg->header.frame_id;
  }

  // Clear old data
  data_.data.clear();

  // Set data time
  data_.time = msg->header.stamp;

  tf2::Transform tf2_transform;
  // Obtaining the transform to get data from source_frame_id_ to base_frame_id_ frame
  if (!getTransform(base_frame_id_, source_frame_id_, tf2_transform)) {
    return;
  }

  // Calculate poses and fill data array
  float angle = msg->angle_min;
  for(size_t i=0; i<msg->ranges.size(); i++) {
    if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max) {
      // Transform point coordinates from source_frame_id_ -> to base_frame_id_
      tf2::Vector3 p_v3_s(
        msg->ranges[i] * std::cos(angle),
        msg->ranges[i] * std::sin(angle),
        0.0);
      tf2::Vector3 p_v3_b = tf2_transform * p_v3_s;

      // Fill data array
      data_.data.push_back({p_v3_b.x(), p_v3_b.y()});
    }
    angle += msg->angle_increment;
  }
}

}  // namespace nav2_collision_monitor
