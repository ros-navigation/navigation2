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
#include "behaviortree_cpp/utils/shared_library.h"

namespace nav2_behavior_tree
{

BehaviorTreeEngine::BehaviorTreeEngine(
  const std::vector<std::string> & plugin_libraries, rclcpp::Node::SharedPtr node)
{
  BT::SharedLibrary loader;
  for (const auto & p : plugin_libraries) {
    factory_.registerFromPlugin(loader.getOSName(p));
  }

  // clock for throttled debug log
  clock_ = node->get_clock();

  // FIXME: the next two line are needed for back-compatibility with BT.CPP 3.8.x
  // Note that the can be removed, once we migrate from BT.CPP 4.5.x to 4.6+
  BT::ReactiveSequence::EnableException(false);
  BT::ReactiveFallback::EnableException(false);
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
  try {
    while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
      if (cancelRequested()) {
        tree->haltTree();
        return BtStatus::CANCELED;
      }

      result = tree->tickOnce();

      onLoop();

      if (!loopRate.sleep()) {
        RCLCPP_DEBUG_THROTTLE(
          rclcpp::get_logger("BehaviorTreeEngine"),
          *clock_, 1000,
          "Behavior Tree tick rate %0.2f was exceeded!",
          1.0 / (loopRate.period().count() * 1.0e-9));
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BehaviorTreeEngine"),
      "Behavior tree threw exception: %s. Exiting with failure.", ex.what());
    return BtStatus::FAILED;
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

// In order to re-run a Behavior Tree, we must be able to reset all nodes to the initial state
void
BehaviorTreeEngine::haltAllActions(BT::Tree & tree)
{
  // this halt signal should propagate through the entire tree.
  tree.haltTree();
}

}  // namespace nav2_behavior_tree
