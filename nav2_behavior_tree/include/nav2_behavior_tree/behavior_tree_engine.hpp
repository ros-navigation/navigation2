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

#ifndef NAV2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
#define NAV2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"


namespace nav2_behavior_tree
{

enum class BtStatus { SUCCEEDED, FAILED, CANCELED };

class BehaviorTreeEngine
{
public:
  explicit BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries);
  virtual ~BehaviorTreeEngine() {}

  BtStatus run(
    BT::Tree * tree,
    std::function<void()> onLoop,
    std::function<bool()> cancelRequested,
    std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));

  BT::Tree createTreeFromText(
    const std::string & xml_string,
    BT::Blackboard::Ptr blackboard);

  BT::Tree createTreeFromFile(
    const std::string & file_path,
    BT::Blackboard::Ptr blackboard);

  void addGrootMonitoring(
    BT::Tree * tree,
    uint16_t publisher_port,
    uint16_t server_port,
    uint16_t max_msg_per_second = 25);

  void resetGrootMonitor();

  // In order to re-run a Behavior Tree, we must be able to reset all nodes to the initial state
  void haltAllActions(BT::TreeNode * root_node);

protected:
  // The factory that will be used to dynamically construct the behavior tree
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::PublisherZMQ> groot_monitor_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
