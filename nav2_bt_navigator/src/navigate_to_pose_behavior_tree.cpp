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
#include "nav2_tasks/compute_path_to_pose_action.hpp"
#include "nav2_tasks/follow_path_action.hpp"
#include "nav2_tasks/is_localized_condition.hpp"
#include "nav2_tasks/is_stuck_condition.hpp"
#include "nav2_tasks/rate_controller_node.hpp"
#include "nav2_tasks/spin_action.hpp"
#include "nav2_tasks/back_up_action.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace nav2_bt_navigator
{

NavigateToPoseBehaviorTree::NavigateToPoseBehaviorTree()
{
  // Register our custom action nodes so that they can be included in XML description
  factory_.registerNodeType<nav2_tasks::ComputePathToPoseAction>("ComputePathToPose");
  factory_.registerNodeType<nav2_tasks::FollowPathAction>("FollowPath");
  factory_.registerNodeType<nav2_tasks::BackUpAction>("BackUp");
  factory_.registerNodeType<nav2_tasks::SpinAction>("Spin");

  // Register our custom condition nodes
  factory_.registerNodeType<nav2_tasks::IsStuckCondition>("IsStuck");
  factory_.registerNodeType<nav2_tasks::IsLocalizedCondition>("IsLocalized");

  // Register our simple condition nodes
  factory_.registerSimpleCondition("initialPoseReceived",
    std::bind(&NavigateToPoseBehaviorTree::initialPoseReceived, this, std::placeholders::_1));

  // Register our custom decorator nodes
  factory_.registerNodeType<nav2_tasks::RateController>("RateController");

  // Register our simple action nodes
  factory_.registerSimpleAction("globalLocalizationServiceRequest",
    std::bind(&NavigateToPoseBehaviorTree::globalLocalizationServiceRequest, this));

  global_localization_client_ =
    std::make_unique<nav2_util::GlobalLocalizationServiceClient>("bt_navigator");
}

BT::NodeStatus
NavigateToPoseBehaviorTree::globalLocalizationServiceRequest()
{
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto response = std::make_shared<std_srvs::srv::Empty::Response>();

  auto succeeded = global_localization_client_->invoke(request, response);
  return succeeded ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus
NavigateToPoseBehaviorTree::initialPoseReceived(BT::TreeNode & tree_node)
{
  auto initPoseReceived = tree_node.blackboard()->template get<bool>("initial_pose_received");
  return initPoseReceived ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace nav2_bt_navigator
