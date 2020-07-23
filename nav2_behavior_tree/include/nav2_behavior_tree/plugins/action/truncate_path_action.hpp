// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_ACTION_HPP_

#include <memory>
#include <string>

#include "nav_msgs/msg/path.hpp"

#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

class TruncatePath : public BT::ActionNodeBase
{
public:
  TruncatePath(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);


  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "Original Path"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "Path truncated to a certain distance"),
      BT::InputPort<double>("distance", 1.0, "distance"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;

  double distance_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_ACTION_HPP_
