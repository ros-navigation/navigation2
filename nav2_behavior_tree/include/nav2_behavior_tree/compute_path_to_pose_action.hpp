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

#ifndef NAV2_BEHAVIOR_TREE__COMPUTE_PATH_TO_POSE_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__COMPUTE_PATH_TO_POSE_ACTION_HPP_

#include <memory>
#include <string>

#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{

class ComputePathToPoseAction : public BtActionNode<nav2_msgs::action::ComputePathToPose>
{
public:
  explicit ComputePathToPoseAction(const std::string & action_name)
  : BtActionNode<nav2_msgs::action::ComputePathToPose>(action_name)
  {
  }

  void on_tick() override
  {
    goal_.pose = *(blackboard()->get<geometry_msgs::msg::PoseStamped::SharedPtr>("goal"));
  }

  void on_success() override
  {
    *(blackboard()->get<nav2_msgs::msg::Path::SharedPtr>("path")) = result_.result->path;

    if (first_time_) {
      first_time_ = false;
    } else {
      blackboard()->set<bool>("path_updated", true);  // NOLINT
    }
  }

private:
  bool first_time_{true};
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__COMPUTE_PATH_TO_POSE_ACTION_HPP_
