// Copyright (c) 2022 Pradheep Padmanabhan - Neobotix GmbH
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONTROLLER_CANCEL_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONTROLLER_CANCEL_NODE_HPP_

#include <memory>
#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"

#include "lifecycle_msgs/srv/change_state.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief The ControllerCancel behavior is used for the transtition of the controller
 * to the shutdown lifecycle state.
 */
class ControllerCancel : public BtServiceNode<lifecycle_msgs::srv::ChangeState>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::ControllerCancel
   *
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf  BT node configuration
   */
  ControllerCancel(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONTROLLER_CANCEL_NODE_HPP_
