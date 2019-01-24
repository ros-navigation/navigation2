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

#ifndef NAV2_WORLD_MODEL__WORLD_REPRESENTATION_HPP_
#define NAV2_WORLD_MODEL__WORLD_REPRESENTATION_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav2_msgs/srv/process_region.hpp"

namespace nav2_world_model
{

using nav2_msgs::srv::GetCostmap;
using nav2_msgs::srv::ProcessRegion;

class WorldRepresentation
{
public:
  explicit WorldRepresentation(const std::string & name, rclcpp::Node::SharedPtr & node)
  : name_(name), node_(node)
  {
  }

  WorldRepresentation() = delete;

  virtual GetCostmap::Response getCostmap(const GetCostmap::Request & request) = 0;

  // Verify if a region is unoccupied
  virtual ProcessRegion::Response confirmFreeSpace(const ProcessRegion::Request & request) = 0;

  virtual ~WorldRepresentation() {}

protected:
  std::string name_;

  // The ROS node to use to create publishers and subscribers
  rclcpp::Node::SharedPtr node_;
};

}  // namespace nav2_world_model

#endif  // NAV2_WORLD_MODEL__WORLD_REPRESENTATION_HPP_
