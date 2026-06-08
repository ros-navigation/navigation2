// Copyright (c) 2022 Neobotix GmbH
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

#include <string>
#include <memory>
#include <vector>
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/path_utils.hpp"

#include "nav2_behavior_tree/plugins/decorator/path_longer_on_approach.hpp"

namespace nav2_behavior_tree
{

PathLongerOnApproach::PathLongerOnApproach(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
}

bool PathLongerOnApproach::isRobotInGoalProximity(
  const nav_msgs::msg::Path & old_path,
  double & prox_leng)
{
  return nav2_util::geometry_utils::calculate_path_length(old_path, 0) < prox_leng;
}

bool PathLongerOnApproach::isNewPathLonger(
  const nav_msgs::msg::Path & new_path,
  const nav_msgs::msg::Path & old_path,
  double & length_factor)
{
  return nav2_util::geometry_utils::calculate_path_length(new_path, 0) >
         length_factor * nav2_util::geometry_utils::calculate_path_length(
    old_path, 0);
}

inline BT::NodeStatus PathLongerOnApproach::tick()
{
  getInput("path", new_path_ptr_);
  getInput("prox_len", prox_len_);
  getInput("length_factor", length_factor_);

  if (!new_path_ptr_) {
    return BT::NodeStatus::SUCCESS;
  }

  if (first_time_ == false) {
    if (!old_path_ptr_ || old_path_ptr_->poses.empty() || new_path_ptr_->poses.empty() ||
      old_path_ptr_->poses.back().pose != new_path_ptr_->poses.back().pose)
    {
      first_time_ = true;
    }
  }
  setStatus(BT::NodeStatus::RUNNING);

  // Check if the path is updated and valid, compare the old and the new path length,
  // given the goal proximity and check if the new path is longer
  if (old_path_ptr_ &&
    nav2_util::isPathUpdated(*new_path_ptr_, *old_path_ptr_) &&
    isRobotInGoalProximity(*old_path_ptr_, prox_len_) &&
    isNewPathLonger(*new_path_ptr_, *old_path_ptr_, length_factor_) && !first_time_)
  {
    const BT::NodeStatus child_state = child_node_->executeTick();
    switch (child_state) {
      case BT::NodeStatus::SKIPPED:
      case BT::NodeStatus::RUNNING:
        return child_state;
      case BT::NodeStatus::SUCCESS:
      case BT::NodeStatus::FAILURE:
        old_path_ptr_ = new_path_ptr_;  // shared_ptr copy — no path data copied
        resetChild();
        return child_state;
      default:
        old_path_ptr_ = new_path_ptr_;  // shared_ptr copy — no path data copied
        return BT::NodeStatus::FAILURE;
    }
  }
  old_path_ptr_ = new_path_ptr_;  // shared_ptr copy — no path data copied
  first_time_ = false;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathLongerOnApproach>("PathLongerOnApproach");
}
