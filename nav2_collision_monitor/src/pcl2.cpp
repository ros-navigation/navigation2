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

#include "nav2_collision_monitor/pcl2.hpp"

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace nav2_collision_monitor
{

PCL2::PCL2(
  nav2_util::LifecycleNode * node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string source_topic,
  const std::string base_frame_id,
  const double transform_tolerance,
  const double max_time_shift,
  const double min_z, const double max_z)
: min_z_(min_z), max_z_(max_z)
{
  node_ = node;
  RCLCPP_INFO(node_->get_logger(), "Creating PCL2");

  tf_buffer_ = tf_buffer;

  source_type_ = PCL_2;
  source_topic_ = source_topic;
  base_frame_id_ = base_frame_id;

  transform_tolerance_ = transform_tolerance;
  max_time_shift_ = max_time_shift;

  rclcpp::QoS pcl2_qos = rclcpp::SensorDataQoS();  // set to default
  data_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    source_topic_, pcl2_qos,
    std::bind(&PCL2::dataCallback, this, std::placeholders::_1));
}

PCL2::~PCL2() {
  RCLCPP_INFO(node_->get_logger(), "Destroying PCL2");

  tf_buffer_.reset();

  data_sub_.reset();
}

void PCL2::dataCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
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

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  // Fill data array with PCL points in base_frame_id_
  for (int i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, i++) {
    // Transform point coordinates from source_frame_id_ -> to base_frame_id_
    tf2::Vector3 p_v3_s(*iter_x, *iter_y, *iter_z);
    tf2::Vector3 p_v3_b = tf2_transform * p_v3_s;

    // Fill data array
    if (p_v3_b.z() >= min_z_ && p_v3_b.z() <= max_z_) {
      data_.data.push_back({p_v3_b.x(), p_v3_b.y()});
    }
  }
}

}  // namespace nav2_collision_monitor
