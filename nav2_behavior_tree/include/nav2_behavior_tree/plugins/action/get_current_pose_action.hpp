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
#include <vector>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

#include "behaviortree_cpp/action_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase to shorten path by some distance
 */
class GetCurrentPoseAction : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::GetCurrentPoseAction constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  GetCurrentPoseAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("global_frame", "Global reference frame"),
      BT::InputPort<std::string>("robot_base_frame", "robot base frame"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_pose", "Current pose output"),
    };
  }

private:
  /**
   * @brief The other (optional) override required by a BT action.
   */
  void halt() override {}

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  std::string global_frame_, robot_base_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  double transform_tolerance_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GET_CURRENT_POSE_ACTION_HPP_
