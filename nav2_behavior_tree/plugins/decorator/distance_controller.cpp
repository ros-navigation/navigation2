// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__DISTANCE_CONTROLLER_HPP_
#define NAV2_BEHAVIOR_TREE__DISTANCE_CONTROLLER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <cmath>

#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

#include "behaviortree_cpp_v3/decorator_node.h"

namespace nav2_behavior_tree
{

class DistanceController : public BT::DecoratorNode
{
public:
  DistanceController(
    const std::string & name,
    const BT::NodeConfiguration & conf)
  : BT::DecoratorNode(name, conf),
    distance_(1.0)
  {
    getInput("distance", distance_);
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  }

  // Any BT node that accepts parameters must provide a requiredNodeParameters method
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", 1.0, "Distance")
    };
  }

private:
  BT::NodeStatus tick() override;

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
};

static bool first_time{false};

inline BT::NodeStatus DistanceController::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    // Reset the starting position since we're starting a new iteration of
    // the distance controller (moving from IDLE to RUNNING)
    if (!nav2_util::getCurrentPose(start_pose_, *tf_)) {
      RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
      return BT::NodeStatus::FAILURE;
    }
    first_time = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  // Determine distance travelled since we've started this iteration
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(current_pose, *tf_)) {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }

  // Get euclidean distance
  auto travelled = euclidean_distance(start_pose_, current_pose);

  // The child gets ticked the first time through and every time the threshold
  // distance is crossed. In addition, once the child begins to run, it is
  // ticked each time 'til completion
  if (first_time || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    travelled >= distance_)
  {
    first_time = false;
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        child_node_->setStatus(BT::NodeStatus::IDLE);
        if (!nav2_util::getCurrentPose(start_pose_, *tf_)) {
          RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
          return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        child_node_->setStatus(BT::NodeStatus::IDLE);
        return BT::NodeStatus::FAILURE;
    }
  }

  return status();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::DistanceController>("DistanceController");
}


#endif  // NAV2_BEHAVIOR_TREE__DISTANCE_CONTROLLER_HPP_
