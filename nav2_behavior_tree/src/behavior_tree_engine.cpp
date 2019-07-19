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

#include "nav2_behavior_tree/behavior_tree_engine.hpp"

#include <memory>
#include <string>

#include "behaviortree_cpp/blackboard/blackboard_local.h"
#include "nav2_behavior_tree/back_up_action.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav2_behavior_tree/compute_path_to_pose_action.hpp"
#include "nav2_behavior_tree/follow_path_action.hpp"
#include "nav2_behavior_tree/goal_reached_condition.hpp"
#include "nav2_behavior_tree/is_localized_condition.hpp"
#include "nav2_behavior_tree/is_stuck_condition.hpp"
#include "nav2_behavior_tree/rate_controller_node.hpp"
#include "nav2_behavior_tree/recovery_node.hpp"
#include "nav2_behavior_tree/spin_action.hpp"
#include "nav2_util/clear_entirely_costmap_service_client.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace nav2_behavior_tree
{

BehaviorTreeEngine::BehaviorTreeEngine()
{
  // Register our custom action nodes so that they can be included in XML description
  factory_.registerNodeType<nav2_behavior_tree::ComputePathToPoseAction>("ComputePathToPose");
  factory_.registerNodeType<nav2_behavior_tree::FollowPathAction>("FollowPath");
  factory_.registerNodeType<nav2_behavior_tree::BackUpAction>("BackUp");
  factory_.registerNodeType<nav2_behavior_tree::SpinAction>("Spin");

  // Register our custom condition nodes
  factory_.registerNodeType<nav2_behavior_tree::IsStuckCondition>("IsStuck");
  factory_.registerNodeType<nav2_behavior_tree::IsLocalizedCondition>("IsLocalized");
  factory_.registerNodeType<nav2_behavior_tree::GoalReachedCondition>("GoalReached");

  // Register our simple condition nodes
  factory_.registerSimpleCondition("initialPoseReceived",
    std::bind(&BehaviorTreeEngine::initialPoseReceived, this, std::placeholders::_1));

  // Register our custom decorator nodes
  factory_.registerNodeType<nav2_behavior_tree::RateController>("RateController");

  // Register our custom control nodes
  factory_.registerNodeType<nav2_behavior_tree::RecoveryNode>("RecoveryNode");

  // Register our simple action nodes
  factory_.registerSimpleAction("globalLocalizationServiceRequest",
    std::bind(&BehaviorTreeEngine::globalLocalizationServiceRequest, this));

  factory_.registerSimpleAction("clearEntirelyCostmapServiceRequest",
    std::bind(&BehaviorTreeEngine::clearEntirelyCostmapServiceRequest, this,
    std::placeholders::_1));

  global_localization_client_ =
    std::make_unique<nav2_util::GlobalLocalizationServiceClient>("bt_navigator");
}

BtStatus
BehaviorTreeEngine::run(
  BT::Blackboard::Ptr & blackboard,
  const std::string & behavior_tree_xml,
  std::function<void()> onLoop,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  // Parse the input XML and create the corresponding Behavior Tree
  BT::Tree tree = BT::buildTreeFromText(factory_, behavior_tree_xml, blackboard);

  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // Loop until something happens with ROS or the node completes
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    if (cancelRequested()) {
      tree.root_node->halt();
      return BtStatus::CANCELED;
    }

    onLoop();

    result = tree.root_node->executeTick();

    loopRate.sleep();
  }

  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BtStatus
BehaviorTreeEngine::run(
  std::unique_ptr<BT::Tree> & tree,
  std::function<void()> onLoop,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // Loop until something happens with ROS or the node completes
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    if (cancelRequested()) {
      tree->root_node->halt();
      return BtStatus::CANCELED;
    }

    onLoop();

    result = tree->root_node->executeTick();

    loopRate.sleep();
  }

  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BT::Tree
BehaviorTreeEngine::buildTreeFromText(std::string & xml_string, BT::Blackboard::Ptr blackboard)
{
  return BT::buildTreeFromText(factory_, xml_string, blackboard);
}

BT::NodeStatus
BehaviorTreeEngine::globalLocalizationServiceRequest()
{
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto response = std::make_shared<std_srvs::srv::Empty::Response>();

  auto succeeded = global_localization_client_->invoke(request, response);
  return succeeded ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus
BehaviorTreeEngine::initialPoseReceived(BT::TreeNode & tree_node)
{
  auto initPoseReceived = tree_node.blackboard()->template get<bool>("initial_pose_received");
  return initPoseReceived ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus 
BehaviorTreeEngine::clearEntirelyCostmapServiceRequest(
  BT::TreeNode & tree_node)
{
  std::string service_name = "/local_costmap/clear_entirely_local_costmap";
  tree_node.getParam<std::string>("service_name", service_name);

  nav2_util::ClearEntirelyCostmapServiceClient clear_entirely_costmap(service_name);
  auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  try {
    clear_entirely_costmap.wait_for_service(std::chrono::seconds(3));
    auto result = clear_entirely_costmap.invoke(request, std::chrono::seconds(3));
    return BT::NodeStatus::SUCCESS;
  } catch (std::runtime_error & e) {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace nav2_behavior_tree
