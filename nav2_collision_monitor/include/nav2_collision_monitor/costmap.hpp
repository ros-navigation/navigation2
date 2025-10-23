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
#ifndef NAV2_COLLISION_MONITOR__COSTMAP_HPP_
#define NAV2_COLLISION_MONITOR__COSTMAP_HPP_

#include <memory>
#include <string>
#include <vector>
#include "nav2_collision_monitor/source.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include <nav2_ros_common/lifecycle_node.hpp>
#include <nav2_ros_common/node_utils.hpp>

namespace nav2_collision_monitor
{

class CostmapSource : public Source
{
public:
  CostmapSource(

    const nav2::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout,
    const bool base_shift_correction);

  ~CostmapSource();

  void configure();

  bool getData(
    const rclcpp::Time & curr_time,
    std::vector<Point> & data) override;

  void getParameters(std::string & source_topic);

private:
  void dataCallback(nav2_msgs::msg::Costmap::ConstSharedPtr msg);
  // ↑ Store the latest Costmap message; we’ll read it in getData()

  nav2_msgs::msg::Costmap::ConstSharedPtr data_;     // Latest costmap message (thread-safe shared ptr)
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr data_sub_;

  // Threshold for considering a cell as an obstacle. Valid range: 0..255.
  // Typical choices: 253 (inscribed), 254 (lethal). Inflation = 1..252.
  int cost_threshold_{253};

   // Whether 255 (NO_INFORMATION) should be treated as an obstacle.
  bool treat_unknown_as_obstacle_{true};


};

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__COSTMAP_HPP_
