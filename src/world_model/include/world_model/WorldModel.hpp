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

#ifndef WORLD_MODEL__WORLDMODEL_HPP_
#define WORLD_MODEL__WORLDMODEL_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav2_msgs/msg/costmap.hpp"

class WorldModel : public rclcpp::Node
{
public:
  explicit WorldModel(const std::string& name);

private:
  // Server for providing a costmap
  rclcpp::Service<nav2_msgs::srv::GetCostmap>::SharedPtr costmapServer_;

  void getCostmap(nav2_msgs::msg::Costmap& costmap);

  void getCostVector(const int width, const int height, std::vector<uint8_t>& data);
};

#endif // WORLD_MODEL__WORLDMODEL_HPP_
