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

#ifndef COSTMAP__COSTMAP_HPP_
#define COSTMAP__COSTMAP_HPP_

#include <vector>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/msg/costmap_meta_data.hpp"

class Costmap
{
public:
  typedef uint8_t CostValue;

  explicit Costmap(rclcpp::Node * node);

  nav2_msgs::msg::Costmap getCostmap(const nav2_msgs::msg::CostmapMetaData & specifications);

  // Mapping for often used cost values
  static const CostValue no_information;
  static const CostValue lethal_obstacle;
  static const CostValue inscribed_inflated_obstacle;
  static const CostValue medium_cost;
  static const CostValue free_space;

private:
  // TODO(orduno): For now, the Costmap isn't itself a node
  rclcpp::Node * node_;

  std::vector<uint8_t> getTestData(const int size_x, const int size_y);
};

#endif  // COSTMAP__COSTMAP_HPP_
