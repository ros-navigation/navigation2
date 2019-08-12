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
// limitations under the License. Reserved.

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_recoveries/back_up.hpp"
#include "nav2_recoveries/spin.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto recoveries_node = rclcpp::Node::make_shared("recoveries");

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(recoveries_node->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    recoveries_node->get_node_base_interface(),
    recoveries_node->get_node_timers_interface());
  tf_buffer->setCreateTimerInterface(timer_interface);
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  recoveries_node->declare_parameter(
    "costmap_topic", rclcpp::ParameterValue(std::string("local_costmap/costmap_raw")));
  recoveries_node->declare_parameter(
    "footprint_topic", rclcpp::ParameterValue(std::string("local_costmap/published_footprint")));

  auto spin = std::make_shared<nav2_recoveries::Spin>(
    recoveries_node, tf_buffer);

  auto back_up = std::make_shared<nav2_recoveries::BackUp>(
    recoveries_node, tf_buffer);

  rclcpp::spin(recoveries_node);
  rclcpp::shutdown();

  return 0;
}
