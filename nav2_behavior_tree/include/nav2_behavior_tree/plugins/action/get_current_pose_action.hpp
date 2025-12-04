// Copyright (c) 2025 Open Navigation LLC
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GET_CURRENT_POSE_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GET_CURRENT_POSE_ACTION_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief Action Node to get the current robot pose from TF
 */
class GetCurrentPoseAction : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  GetCurrentPoseAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("global_frame", "map", "Global reference frame"),
      BT::InputPort<std::string>("robot_base_frame", "base_link", "Robot base frame"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_pose", "Current pose output"),
    };
  }

  /**
   * @brief Main loop
   * @return BT::NodeStatus
   */
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::string global_frame_;
  std::string robot_base_frame_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GET_CURRENT_POSE_ACTION_HPP_
