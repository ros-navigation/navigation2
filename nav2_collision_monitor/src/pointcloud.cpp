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
  const std::string & source_name,
  const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance,
  const tf2::Duration & data_timeout)
: SourceBase(node, tf_buffer, source_name, base_frame_id, transform_tolerance, data_timeout),
  min_height_(0.0), max_height_(0.0), data_(nullptr)
{
  RCLCPP_INFO(logger_, "[%s]: Creating PointCloud", source_name_.c_str());
}

PointCloud::~PointCloud() {
  RCLCPP_INFO(logger_, "[%s]: Destroying PointCloud", source_name_.c_str());

  tf_buffer_.reset();
  data_sub_.reset();
}

void PointCloud::configure()
{
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

void PointCloud::getData(const rclcpp::Time & curr_time, std::vector<Point> & data)
{
  // Ignore data from the source if it is not being published yet or
  // not published for a long time
  if (data_ == nullptr || !sourceValid(curr_time)) {
    return;
  }

  // Obtaining the transform to get data from source_frame_id_ and time where it was obtained
  // to base_frame_id_ frame and current time
  tf2::Transform tf2_transform;
  if (!getSourceBaseTransform(curr_time, data_stamp_, tf2_transform)) {
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*data_, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*data_, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*data_, "z");

  // Refill data array with PointCloud points in base_frame_id_
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    // Transform point coordinates from source_frame_id_ -> to base_frame_id_
    tf2::Vector3 p_v3_s(*iter_x, *iter_y, *iter_z);
    tf2::Vector3 p_v3_b = tf2_transform * p_v3_s;

    // Refill data array
    if (p_v3_b.z() >= min_height_ && p_v3_b.z() <= max_height_) {
      data.push_back({p_v3_b.x(), p_v3_b.y()});
    }
  }
}

void PointCloud::getParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  SourceBase::getParameters(source_topic);

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".min_height", rclcpp::ParameterValue(0.05));
  min_height_ = node->get_parameter(source_name_ + ".min_height").as_double();
  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".max_height", rclcpp::ParameterValue(0.5));
  max_height_ = node->get_parameter(source_name_ + ".max_height").as_double();
}

void PointCloud::dataCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Obtaining source_frame_id_ from first message
  if (source_frame_id_.length() == 0) {
    source_frame_id_ = msg->header.frame_id;
  }

  data_ = msg;
  data_stamp_ = node->now();
}

}  // namespace nav2_collision_monitor
