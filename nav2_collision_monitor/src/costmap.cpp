// Copyright (c) 2025 Angsa Robotics
// Copyright (c) 2025 lotusymt
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

#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_collision_monitor/costmap.hpp"
#include <cmath>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <nav2_ros_common/node_utils.hpp>
#include <nav2_ros_common/qos_profiles.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

namespace nav2_collision_monitor
{

CostmapSource::CostmapSource(
  const nav2::LifecycleNode::WeakPtr & node,
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
  RCLCPP_INFO(logger_, "[%s]: Creating CostmapSource", source_name_.c_str());
}

CostmapSource::~CostmapSource()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying CostmapSource", source_name_.c_str());
  data_sub_.reset();
}

void CostmapSource::configure()
{
  Source::configure();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  std::string source_topic;
  getParameters(source_topic);
  data_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
    source_topic,
    std::bind(&CostmapSource::dataCallback, this, std::placeholders::_1),
    nav2::qos::StandardTopicQoS());
}

bool CostmapSource::getData(
  const rclcpp::Time & curr_time,
  std::vector<Point> & data)
{
  auto node = node_.lock();
  if (data_ == nullptr) {
    return false;
  }

  if (!sourceValid(data_->header.stamp, curr_time)) {
    return false;
  }
  tf2::Transform tf_transform; tf_transform.setIdentity();
  const std::string src = data_->header.frame_id;

  if (src != base_frame_id_) {
    if (!getTransform(curr_time, data_->header, tf_transform)) {
      RCLCPP_WARN(logger_, "[%s] TF %s->%s unavailable at t=%.3f",
        source_name_.c_str(), src.c_str(), base_frame_id_.c_str(), curr_time.seconds());
      return false;
    }
  }

  // Extract lethal/inscribed cells and transform to base frame
  const auto & cm = *data_;
  const auto & meta = cm.metadata;

  for (unsigned int y = 0; y < meta.size_y; ++y) {
    for (unsigned int x = 0; x < meta.size_x; ++x) {
      const int idx = y * meta.size_x + x;
      const uint8_t cell_cost = cm.data[idx];
      const bool is_obstacle =
        (cell_cost >= cost_threshold_ && cell_cost < nav2_costmap_2d::NO_INFORMATION) ||
        (treat_unknown_as_obstacle_ && cell_cost == nav2_costmap_2d::NO_INFORMATION);
      if (is_obstacle) {
        const double wx = meta.origin.position.x + (x + 0.5) * meta.resolution;
        const double wy = meta.origin.position.y + (y + 0.5) * meta.resolution;
        tf2::Vector3 p_v3_s(wx, wy, 0.0);
        tf2::Vector3 p_v3_b = tf_transform * p_v3_s;
        data.push_back({p_v3_b.x(), p_v3_b.y()});
        RCLCPP_DEBUG_THROTTLE(
          logger_, *node->get_clock(), 2000 /*ms*/,
          "[%s] Found obstacles in costmap", source_name_.c_str());
      }
    }
  }

  return true;
}

void CostmapSource::getParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  getCommonParameters(source_topic);

  // Cost threshold (0–255). 253 = inscribed 254 = lethal; 255 = NO_INFORMATION.
  const auto thresh_name = source_name_ + ".cost_threshold";
  // Minimal change (no range descriptor)
  int v = node->declare_or_get_parameter<int>(thresh_name, 253);  // declare if missing, else get
  v = std::max(0, std::min(255, v));  // clamp
  if (v != node->get_parameter(thresh_name).as_int()) {
    RCLCPP_WARN(node->get_logger(), "Clamping %s to %d", thresh_name.c_str(), v);
  }
  cost_threshold_ = v;

  // Whether 255 (NO_INFORMATION) should be treated as an obstacle.
  const auto unk_name = source_name_ + ".treat_unknown_as_obstacle";
  treat_unknown_as_obstacle_ = node->declare_or_get_parameter<bool>(unk_name, true);
}

void CostmapSource::dataCallback(nav2_msgs::msg::Costmap::ConstSharedPtr msg)
{
  data_ = msg;
}

}  // namespace nav2_collision_monitor
