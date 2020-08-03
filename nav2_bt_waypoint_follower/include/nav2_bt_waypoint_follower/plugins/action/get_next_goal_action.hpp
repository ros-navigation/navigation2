// Copyright (c) 2020 ymd-stella
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

#ifndef NAV2_BT_WAYPOINT_FOLLOWER__PLUGINS__ACTION__GET_NEXT_GOAL_ACTION_HPP_
#define NAV2_BT_WAYPOINT_FOLLOWER__PLUGINS__ACTION__GET_NEXT_GOAL_ACTION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_bt_waypoint_follower
{

class GetNextGoalAction : public BT::SyncActionNode
{
public:
  GetNextGoalAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  GetNextGoalAction() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "goals",
        "Destinations to plan to"),
      BT::BidirectionalPort<bool>("goal_achieved", "Has the goal been achieved?"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to")};
  }
};

}  // namespace nav2_bt_waypoint_follower

#endif  // NAV2_BT_WAYPOINT_FOLLOWER__PLUGINS__ACTION__GET_NEXT_GOAL_ACTION_HPP_
