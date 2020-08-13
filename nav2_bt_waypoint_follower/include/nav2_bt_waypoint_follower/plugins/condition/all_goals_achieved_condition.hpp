// Copyright (c) 2020 Samsung Research America
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

#ifndef NAV2_BT_WAYPOINT_FOLLOWER__PLUGINS__CONDITION__ALL_GOALS_ACHIEVED_CONDITION_HPP_
#define NAV2_BT_WAYPOINT_FOLLOWER__PLUGINS__CONDITION__ALL_GOALS_ACHIEVED_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_bt_waypoint_follower
{

class AllGoalsAchievedCondition : public BT::ConditionNode
{
public:
  AllGoalsAchievedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  AllGoalsAchievedCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("goal_achieved", "Has the goal been achieved?"), };
  }
};

}  // namespace nav2_bt_waypoint_follower

#endif  // NAV2_BT_WAYPOINT_FOLLOWER__PLUGINS__CONDITION__ALL_GOALS_ACHIEVED_CONDITION_HPP_
