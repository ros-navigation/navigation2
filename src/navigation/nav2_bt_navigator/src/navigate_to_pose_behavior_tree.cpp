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

#include <memory>
#include "nav2_bt_navigator/navigate_to_pose_behavior_tree.hpp"

using namespace std::chrono_literals;

namespace nav2_bt_navigator
{

NavigateToPoseBehaviorTree::NavigateToPoseBehaviorTree(rclcpp::Node::SharedPtr node)
: node_(node)
{
  // Create the input and output data
  computePathToPoseCommand_ = std::make_shared<nav2_tasks::ComputePathToPoseCommand>();
  computePathToPoseResult_ = std::make_shared<nav2_tasks::ComputePathToPoseResult>();

  followPathCommand_ = std::make_shared<nav2_tasks::FollowPathCommand>();
  followPathResult_ = std::make_shared<nav2_tasks::FollowPathResult>();

  // Create the nodes of the tree
  root_ = std::make_unique<BT::SequenceNodeWithMemory>("Sequence");

  firstPath_ = std::make_unique<nav2_tasks::ComputePathToPoseAction>(
    node_, "FirstPath",
    computePathToPoseCommand_, computePathToPoseResult_);

  sel_ = std::make_unique<BT::FallbackNode>("Fallback");
  reachedGoalNode_ = std::make_unique<ReachedGoalConditionNode>(node);
  parNode_ = std::make_unique<BT::ParallelNode>("Parallel", 3); // TODO: Will never fire

  computePathToPoseAction_ = std::make_unique<nav2_tasks::ComputePathToPoseAction>(
    node_, "ComputePathToPoseAction",
    computePathToPoseCommand_, computePathToPoseResult_);

  followPathAction_ = std::make_unique<nav2_tasks::FollowPathAction>(
    node_, "FollowPathAction",
    followPathCommand_, followPathResult_);

  // Add the nodes to the tree, creating the tree structure
  root_->AddChild(firstPath_.get());
  root_->AddChild(sel_.get());
  sel_->AddChild(reachedGoalNode_.get());
  sel_->AddChild(parNode_.get());
  parNode_->AddChild(computePathToPoseAction_.get());
  parNode_->AddChild(followPathAction_.get());
}

nav2_tasks::TaskStatus
NavigateToPoseBehaviorTree::run(
  nav2_tasks::NavigateToPoseCommand::SharedPtr command,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  rclcpp::WallRate loop_rate(loopTimeout);

  // Compose the PathEndPoints message for the Navigation module
  // TODO(mjeronimo): starting pose needs to come from localization
  computePathToPoseCommand_->start = command->pose;
  computePathToPoseCommand_->goal = command->pose;

  BT::ReturnStatus result = root_->get_status();

  while (rclcpp::ok() && !(result == BT::SUCCESS || result == BT::FAILURE)) {
    result = root_->Tick();

    if (cancelRequested()) {
      root_->Halt();
      return nav2_tasks::TaskStatus::CANCELED;
    }

    loop_rate.sleep();
  }

  return (result ==
         BT::SUCCESS) ? nav2_tasks::TaskStatus::SUCCEEDED : nav2_tasks::TaskStatus::FAILED;
}

}  // namespace nav2_bt_navigator
