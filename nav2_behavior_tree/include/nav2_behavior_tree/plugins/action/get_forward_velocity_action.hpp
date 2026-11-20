// Copyright (c) 2026
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GET_FORWARD_VELOCITY_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GET_FORWARD_VELOCITY_ACTION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_util/odometry_utils.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT action that reads the raw forward velocity from the navigator's odometry
 *
 * The output is the signed body-frame x velocity in m/s. The action fails if the
 * odometry is unavailable, stale, future-dated, or has a non-finite velocity.
 *
 * Usage in XML:
 * @code
 * <GetForwardVelocity forward_velocity="{forward_velocity}" max_odom_age="1.0"/>
 * @endcode
 */
class GetForwardVelocity : public BT::SyncActionNode
{
public:
  /**
   * @brief A nav2_behavior_tree::GetForwardVelocity constructor
   * @param name Name for the XML tag for this node
   * @param config BT node configuration
   */
  GetForwardVelocity(const std::string & name, const BT::NodeConfiguration & config);

  /**
   * @brief Creates the input and output BT ports
   * @return BT::PortsList Containing the maximum odometry age and forward velocity ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("max_odom_age", 1.0, "Maximum odometry age in seconds"),
      BT::OutputPort<double>("forward_velocity", "Body-frame forward velocity in m/s")
    };
  }

  /**
   * @brief Read the latest raw odometry and output its forward velocity
   * @return BT::NodeStatus SUCCESS if the odometry is valid and fresh, FAILURE otherwise
   */
  BT::NodeStatus tick() override;

private:
  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GET_FORWARD_VELOCITY_ACTION_HPP_
