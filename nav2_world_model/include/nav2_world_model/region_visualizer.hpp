// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_WORLD_MODEL__REGION_VISUALIZER_HPP_
#define NAV2_WORLD_MODEL__REGION_VISUALIZER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace nav2_world_model
{

class RegionVisualizer
{
public:
  explicit RegionVisualizer(rclcpp::Node::SharedPtr & node)
  : node_(node),
    marker_(visualization_msgs::msg::Marker())
  {
    marker_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "world_model_cell", 1);
  }

  void addFreeCell(const double x, const double y, const double z)
  {
    marker_.points.push_back(makePoint(x, y, z));

    std_msgs::msg::ColorRGBA color;
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
    color.a = 0.3;
    marker_.colors.push_back(color);
  }

  void addOccupiedCell(const double x, const double y, const double z)
  {
    marker_.points.push_back(makePoint(x, y, z));

    std_msgs::msg::ColorRGBA color;
    color.r = 1.0;
    color.g = 1.0;
    color.b = 0.0;
    color.a = 0.3;
    marker_.colors.push_back(color);
  }

  void clearMarker()
  {
    marker_.points.clear();
    marker_.colors.clear();
  }

  void publish(const double cell_size)
  {
    if (marker_.points.empty() || marker_.colors.empty()) {
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "Publishing %d markers describing a region",
      marker_.points.size());

    marker_.header.frame_id = "map";
    // marker_.header.stamp = node_.now();
    builtin_interfaces::msg::Time time;
    time.sec = 0;
    time.nanosec = 0;
    marker_.header.stamp = time;

    marker_.ns = "world_model_cell";
    static int index = 0;
    marker_.id = index;

    marker_.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker_.action = visualization_msgs::msg::Marker::ADD;

    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker_.scale.x = cell_size;
    marker_.scale.y = cell_size;
    marker_.scale.z = cell_size;

    // Duration of zero indicates the object should last forever
    builtin_interfaces::msg::Duration duration;
    duration.sec = 4;
    duration.nanosec = 0;

    marker_.lifetime = duration;
    marker_.frame_locked = false;

    marker_publisher_->publish(marker_);

    clearMarker();
  }

private:
  // ROS node used to create the publishers and subscribers
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

  // Using a marker to visualize a region of cells on RVIZ
  visualization_msgs::msg::Marker marker_;

  geometry_msgs::msg::Point makePoint(
    const double x, const double y, const double z) const
  {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;

    return point;
  }
};

}  // namespace nav2_world_model

#endif  // NAV2_WORLD_MODEL__REGION_VISUALIZER_HPP_