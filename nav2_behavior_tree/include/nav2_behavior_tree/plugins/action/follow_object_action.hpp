// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_OBJECT_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_OBJECT_ACTION_HPP_

#include <string>
#include <memory>
#include <vector>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/follow_object.hpp"


namespace nav2_behavior_tree
{

/**
 * @brief nav2_behavior_tree::BtActionNode class that wraps nav2_msgs/FollowObject
 */
class FollowObjectAction
  : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::FollowObject>
{
  using Action = nav2_msgs::action::FollowObject;
  using ActionResult = Action::Result;

public:
  /**
   * @brief A constructor for nav2_behavior_tree::FollowObjectAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  FollowObjectAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Function to perform some user-defined operation upon abortion of the action
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief Function to perform some user-defined operation upon cancellation of the action
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>(
          "pose_topic", "dynamic_pose", "Topic to publish the pose of the object to follow"),
        BT::InputPort<std::string>(
          "tracked_frame", "Target frame to follow (Optional, used if pose_topic is not set)"),
        BT::InputPort<float>(
          "max_duration", 0.0, "The maximum duration to follow the object (Optional)"),

        BT::OutputPort<ActionResult::_total_elapsed_time_type>(
          "total_elapsed_time", "Total elapsed time"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "Error code"),
        BT::OutputPort<std::string>(
          "error_msg", "Error message"),
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_OBJECT_ACTION_HPP_
