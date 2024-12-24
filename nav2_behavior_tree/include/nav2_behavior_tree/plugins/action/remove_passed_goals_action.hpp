// Copyright (c) 2021 Samsung Research America
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_

#include <vector>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "behaviortree_cpp/action_node.h"
#include "nav2_behavior_tree/bt_utils.hpp"

namespace nav2_behavior_tree
{

class RemovePassedGoals : public BT::ActionNodeBase
{
public:
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  RemovePassedGoals(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goals>("input_goals", "Original goals to remove viapoints from"),
      BT::OutputPort<Goals>("output_goals", "Goals with passed viapoints removed"),
      BT::InputPort<double>("radius", 0.5, "radius to goal for it to be considered for removal"),
      BT::InputPort<std::string>("global_frame", "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", "Robot base frame"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;

  double viapoint_achieved_radius_;
  double transform_tolerance_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string robot_base_frame_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_
