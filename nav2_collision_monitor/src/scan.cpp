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
  const std::string & source_name,
  const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance,
  const tf2::Duration & data_timeout)
: SourceBase(node, tf_buffer, source_name, base_frame_id, transform_tolerance, data_timeout)
{
  RCLCPP_INFO(logger_, "[%s]: Creating Scan", source_name_.c_str());
}

Scan::~Scan() {
  RCLCPP_INFO(logger_, "[%s]: Destroying Scan", source_name_.c_str());

  tf_buffer_.reset();
  data_sub_.reset();
}

void Scan::configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;

  // Laser scanner has no its own parameters
  SourceBase::getParameters(source_topic);

  rclcpp::QoS scan_qos = rclcpp::SensorDataQoS();  // set to default
  data_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    source_topic, scan_qos,
    std::bind(&Scan::dataCallback, this, std::placeholders::_1));
}

void Scan::dataCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  // Obtaining source_frame_id_ from first message
  if (source_frame_id_.length() == 0) {
    source_frame_id_ = msg->header.frame_id;
  }

  data_ = msg;
  data_stamp_ = msg->header.stamp;
}

void Scan::getData(std::vector<Point> & data, const rclcpp::Time & curr_time)
{
  // Ignore data from the source if it is not published for a long time
  if (!sourceValid(curr_time)) {
    return;
  }

  // Obtaining the transform to get data from source_frame_id_ and time where it was obtained
  // to base_frame_id_ frame and current time
  tf2::Transform tf2_transform;
  if (!getSourceBaseTransform(curr_time, data_stamp_, tf2_transform)) {
    return;
  }

  // Calculate poses and refill data array
  float angle = data_->angle_min;
  for(size_t i=0; i<data_->ranges.size(); i++) {
    if (data_->ranges[i] >= data_->range_min && data_->ranges[i] <= data_->range_max) {
      // Transform point coordinates from source_frame_id_ -> to base_frame_id_
      tf2::Vector3 p_v3_s(
        data_->ranges[i] * std::cos(angle),
        data_->ranges[i] * std::sin(angle),
        0.0);
      tf2::Vector3 p_v3_b = tf2_transform * p_v3_s;

      // Refill data array
      data.push_back({p_v3_b.x(), p_v3_b.y()});
    }
    angle += data_->angle_increment;
  }
}

}  // namespace nav2_collision_monitor
