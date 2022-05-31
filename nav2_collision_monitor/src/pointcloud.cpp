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

#include "nav2_collision_monitor/pointcloud.hpp"

#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

PointCloud::PointCloud(
  const nav2_util::LifecycleNode::WeakPtr & node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string source_name,
  const std::string base_frame_id,
  const double transform_tolerance,
  const double max_time_shift)
: min_height_(0.0), max_height_(0.0)
{
  node_ = node;
  auto node_sptr = node_.lock();
  if (node_sptr) {
    RCLCPP_INFO(node_sptr->get_logger(), "Creating PointCloud");
  }

  tf_buffer_ = tf_buffer;

  source_type_ = POINTCLOUD;
  source_name_ = source_name;
  base_frame_id_ = base_frame_id;

  transform_tolerance_ = transform_tolerance;
  max_time_shift_ = max_time_shift;
}

PointCloud::~PointCloud() {
  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO(node->get_logger(), "Destroying PointCloud");
  }

  tf_buffer_.reset();

  data_sub_.reset();
}

bool PointCloud::init()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!getParameters()) {
    return false;
  }

  rclcpp::QoS pointcloud_qos = rclcpp::SensorDataQoS();  // set to default
  data_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    source_topic_, pointcloud_qos,
    std::bind(&PointCloud::dataCallback, this, std::placeholders::_1));

  return true;
}

bool PointCloud::getParameters()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!SourceBase::getParameters()) {
    return false;
  }

  try {
    nav2_util::declare_parameter_if_not_declared(
      node, source_name_ + ".min_height", rclcpp::ParameterValue(0.05));
    min_height_ = node->get_parameter(source_name_ + ".min_height").as_double();
    nav2_util::declare_parameter_if_not_declared(
      node, source_name_ + ".max_height", rclcpp::ParameterValue(0.5));
    max_height_ = node->get_parameter(source_name_ + ".max_height").as_double();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      node->get_logger(), "Error while getting pointcloud parameters: %s", ex.what());
    return false;
  }

  return true;
}

void PointCloud::dataCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
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

  // Fill data array with PointCloud points in base_frame_id_
  for (int i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, i++) {
    // Transform point coordinates from source_frame_id_ -> to base_frame_id_
    tf2::Vector3 p_v3_s(*iter_x, *iter_y, *iter_z);
    tf2::Vector3 p_v3_b = tf2_transform * p_v3_s;

    // Fill data array
    if (p_v3_b.z() >= min_height_ && p_v3_b.z() <= max_height_) {
      data_.data.push_back({p_v3_b.x(), p_v3_b.y()});
    }
  }
}

}  // namespace nav2_collision_monitor
