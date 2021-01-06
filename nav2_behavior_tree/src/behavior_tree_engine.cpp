// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Florian Gramss
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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/utils/shared_library.h"

namespace nav2_behavior_tree
{

BehaviorTreeEngine::BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries)
{
  BT::SharedLibrary loader;
  for (const auto & p : plugin_libraries) {
    factory_.registerFromPlugin(loader.getOSName(p));
  }
}

BtStatus
BehaviorTreeEngine::run(
  BT::Tree * tree,
  std::function<void()> onLoop,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // Loop until something happens with ROS or the node completes
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    if (cancelRequested()) {
      tree->rootNode()->halt();
      return BtStatus::CANCELED;
    }

    result = tree->tickRoot();

    onLoop();

    loopRate.sleep();
  }

  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BT::Tree
BehaviorTreeEngine::createTreeFromText(
  const std::string & xml_string,
  BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromText(xml_string, blackboard);
}

BT::Tree
BehaviorTreeEngine::createTreeFromFile(
  const std::string & file_path,
  BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromFile(file_path, blackboard);
}

void
BehaviorTreeEngine::addGrootMonitoring(
  BT::Tree * tree,
  uint16_t publisher_port,
  uint16_t server_port,
  uint16_t max_msg_per_second)
{
  // This logger publish status changes using ZeroMQ. Used by Groot
  groot_monitor_ = std::make_unique<BT::PublisherZMQ>(
    *tree, max_msg_per_second, publisher_port,
    server_port);
}

void
BehaviorTreeEngine::resetGrootMonitor()
{
  groot_monitor_.reset();
}

// In order to re-run a Behavior Tree, we must be able to reset all nodes to the initial state
void
BehaviorTreeEngine::haltAllActions(BT::TreeNode * root_node)
{
  // this halt signal should propagate through the entire tree.
  root_node->halt();

  // but, just in case...
  auto visitor = [](BT::TreeNode * node) {
      if (node->status() == BT::NodeStatus::RUNNING) {
        node->halt();
      }
    };
  BT::applyRecursiveVisitor(root_node, visitor);
}

}  // namespace nav2_behavior_tree
