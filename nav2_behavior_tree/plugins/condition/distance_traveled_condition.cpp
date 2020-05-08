// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#ifndef NAV2_BEHAVIOR_TREE__DISTANCE_TRAVELED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__DISTANCE_TRAVELED_CONDITION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

class DistanceTraveledCondition : public BT::ConditionNode
{
public:
  DistanceTraveledCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf),
    distance_(1.0),
    first_time_(true)
  {
    getInput("distance", distance_);
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  }

  DistanceTraveledCondition() = delete;

  BT::NodeStatus tick() override
  {
    if (first_time_) {
      if (!nav2_util::getCurrentPose(start_pose_, *tf_)) {
        RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
        return BT::NodeStatus::FAILURE;
      }
      first_time_ = false;
      return BT::NodeStatus::SUCCESS;
    }

    // Determine distance travelled since we've started this iteration
    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(current_pose, *tf_)) {
      RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
      return BT::NodeStatus::FAILURE;
    }

    // Get euclidean distance
    auto travelled = euclidean_distance(start_pose_, current_pose);

    if (travelled < distance_) {
      return BT::NodeStatus::FAILURE;
    }

    // Update start pose
    start_pose_ = current_pose;

    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", 1.0, "Distance")
    };
  }

private:
  double euclidean_distance(
    const geometry_msgs::msg::PoseStamped & pose1,
    const geometry_msgs::msg::PoseStamped & pose2)
  {
    const double dx = pose1.pose.position.x - pose2.pose.position.x;
    const double dy = pose1.pose.position.y - pose2.pose.position.y;
    const double dz = pose1.pose.position.z - pose2.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  geometry_msgs::msg::PoseStamped start_pose_;

  double distance_;
  bool first_time_;
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::DistanceTraveledCondition>("DistanceTraveled");
}

#endif  // NAV2_BEHAVIOR_TREE__DISTANCE_TRAVELED_CONDITION_HPP_
