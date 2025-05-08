// Copyright (c) 2025 Open Navigation LLC
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GET_NEXT_FEW_GOALS_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GET_NEXT_FEW_GOALS_ACTION_HPP_

#include <vector>
#include <memory>
#include <string>

#include "nav_msgs/msg/goals.hpp"
#include "nav2_msgs/msg/route.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "behaviortree_cpp/action_node.h"

namespace nav2_behavior_tree
{

class GetNextFewGoals : public BT::ActionNodeBase
{
public:
  GetNextFewGoals(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);


  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Goals>("input_goals", "Input goals for navigation"),
      BT::InputPort<int>("num_goals", "Number of goals to extract"),
      BT::OutputPort<nav_msgs::msg::Goals>("output_goals", "Output goals for navigation")
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GET_NEXT_FEW_GOALS_ACTION_HPP_
