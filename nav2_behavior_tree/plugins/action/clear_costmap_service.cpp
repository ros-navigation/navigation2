// Copyright (c) 2019 Samsung Research America
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

#ifndef NAV2_BEHAVIOR_TREE__CLEAR_COSTMAP_SERVICE_HPP_
#define NAV2_BEHAVIOR_TREE__CLEAR_COSTMAP_SERVICE_HPP_

#include <string>
#include <memory>
#include <cmath>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

namespace nav2_behavior_tree
{

class ClearEntireCostmapService : public BtServiceNode<nav2_msgs::srv::ClearEntireCostmap>
{
public:
  ClearEntireCostmapService(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf)
  : BtServiceNode<nav2_msgs::srv::ClearEntireCostmap>(service_node_name, conf)
  {
  }

  void on_tick() override
  {
    int recovery_count = 0;
    config().blackboard->get<int>("number_recoveries", recovery_count);  // NOLINT
    recovery_count += 1;
    config().blackboard->set<int>("number_recoveries", recovery_count);  // NOLINT
  }
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ClearEntireCostmapService>("ClearEntireCostmap");
}

#endif  // NAV2_BEHAVIOR_TREE__CLEAR_COSTMAP_SERVICE_HPP_
