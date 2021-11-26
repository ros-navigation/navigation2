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

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/create_timer_ros.h"

#include "nav2_behavior_tree/plugins/action/truncate_path_local_action.hpp"

namespace nav2_behavior_tree
{

TruncatePathLocal::TruncatePathLocal(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  tf_buffer_ =
    config().blackboard->template get<std::shared_ptr<tf2_ros::Buffer>>(
    "tf_buffer");
}

inline BT::NodeStatus TruncatePathLocal::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  nav_msgs::msg::Path path;
  double distance_forward, distance_backward;
  double transform_tolerance;
  std::string robot_frame, global_frame;
  geometry_msgs::msg::PoseStamped pose;
  double angular_distance_weight;

  getInput("input_path", path);
  getInput("distance_forward", distance_forward);
  getInput("distance_backward", distance_backward);
  getInput("transform_tolerance", transform_tolerance);
  getInput("robot_frame", robot_frame);
  getInput("global_frame", global_frame);
  getInput("angular_distance_weight", angular_distance_weight);

  if (robot_frame.empty() || global_frame.empty()) {
    if (!getInput("pose", pose)) {
      RCLCPP_ERROR(
        config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
        "No pose specified for %s", name().c_str());
      return BT::NodeStatus::FAILURE;
    }
  } else {
    if (!nav2_util::getCurrentPose(
        pose, *tf_buffer_, global_frame, robot_frame,
        transform_tolerance)) {
      return BT::NodeStatus::FAILURE;
    }
  }

  if (path.poses.empty()) {
    setOutput("output_path", path);
    return BT::NodeStatus::SUCCESS;
  }

  // find the closest pose on the path
  auto current_pose = nav2_util::geometry_utils::min_by(
    path.poses.begin(), path.poses.end(),
    [&pose,
    angular_distance_weight](const geometry_msgs::msg::PoseStamped & ps) {
      return poseDistance(pose, ps, angular_distance_weight);
    });

  // expand forwards to extract desired length
  double length = 0;
  auto end = current_pose - path.poses.begin();
  while (static_cast<int>(end) < static_cast<int>(path.poses.size() - 1) && length < distance_forward) {
    length += std::hypot(
      path.poses[end + 1].pose.position.x - path.poses[end].pose.position.x,
      path.poses[end + 1].pose.position.y - path.poses[end].pose.position.y);
    end++;
  }
  end++;  // end is exclusive

  // expand backwards to extract desired length
  auto begin = current_pose - path.poses.begin();
  length = 0;
  while (begin > 0 && length < distance_backward) {
    length += std::hypot(
      path.poses[begin + 1].pose.position.x -
      path.poses[begin].pose.position.x,
      path.poses[begin + 1].pose.position.y -
      path.poses[begin].pose.position.y);
    begin--;
  }

  path.poses = std::vector<geometry_msgs::msg::PoseStamped>(
    path.poses.begin() + begin, path.poses.begin() + end);
  setOutput("output_path", path);

  return BT::NodeStatus::SUCCESS;
}

double
TruncatePathLocal::poseDistance(
  const geometry_msgs::msg::PoseStamped & pose1,
  const geometry_msgs::msg::PoseStamped & pose2,
  const double angular_distance_weight)
{
  double dx = pose1.pose.position.x - pose2.pose.position.x;
  double dy = pose1.pose.position.y - pose2.pose.position.y;
  tf2::Quaternion q1;
  tf2::convert(pose1.pose.orientation, q1);
  tf2::Quaternion q2;
  tf2::convert(pose2.pose.orientation, q2);
  double da = angular_distance_weight * std::abs(q1.angleShortestPath(q2));
  return std::sqrt(dx * dx + dy * dy + da * da);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::TruncatePathLocal>(
    "TruncatePathLocal");
}
