// Copyright (c) 2022 Joshua Wallace
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__WOULD_A_PLANNER_RECOVERY_HELP_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__WOULD_A_PLANNER_RECOVERY_HELP_CONDITION_HPP_

#include <string>

#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_behavior_tree/plugins/condition/are_error_codes_present_condition.hpp"

namespace nav2_behavior_tree
{

class WouldAPlannerRecoveryHelp : public AreErrorCodesPresent
{
  using Action = nav2_msgs::action::ComputePathToPose;
  using ActionResult = Action::Result;
  using ThroughAction = nav2_msgs::action::ComputePathThroughPoses;
  using ThroughActionResult = ThroughAction::Result;

public:
  WouldAPlannerRecoveryHelp(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  WouldAPlannerRecoveryHelp() = delete;
};

}  // namespace nav2_behavior_tree
#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__WOULD_A_PLANNER_RECOVERY_HELP_CONDITION_HPP_
