// Copyright (c) 2021 Joshua Wallace
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

#include "nav2_behavior_tree/plugins/condition/is_path_valid_condition.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace nav2_behavior_tree
{

IsPathValidCondition::IsPathValidCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  max_cost_(253), consider_unknown_as_obstacle_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = std::make_shared<nav2_util::ServiceClient<nav2_msgs::srv::IsPathValid>>("is_path_valid",
      node_, false);

  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
}

void IsPathValidCondition::initialize()
{
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
  getInput<unsigned int>("max_cost", max_cost_);
  getInput<bool>("consider_unknown_as_obstacle", consider_unknown_as_obstacle_);
}

BT::NodeStatus IsPathValidCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  nav_msgs::msg::Path path;
  getInput("path", path);

  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();

  request->path = path;
  request->max_cost = max_cost_;
  request->consider_unknown_as_obstacle = consider_unknown_as_obstacle_;
  auto response = client_->invoke(request, server_timeout_);
  if (response->is_valid) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsPathValidCondition>("IsPathValid");
}
