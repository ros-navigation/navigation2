// Copyright (c) 2020 Samsung Research America
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TRANSFORM_AVAILABLE_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TRANSFORM_AVAILABLE_CONDITION_HPP_

#include <string>
#include <atomic>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

class TransformAvailableCondition : public BT::ConditionNode
{
public:
  TransformAvailableCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  TransformAvailableCondition() = delete;

  ~TransformAvailableCondition();

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("child", std::string(), "Child frame for transform"),
      BT::InputPort<std::string>("parent", std::string(), "parent frame for transform")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::atomic<bool> was_found_;

  std::string child_frame_;
  std::string parent_frame_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TRANSFORM_AVAILABLE_CONDITION_HPP_
