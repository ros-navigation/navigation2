// Copyright (c) 2018 Intel Corporation
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

#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/back_up_directional_action.hpp"

namespace nav2_behavior_tree
{

BackUpDirectionalAction::BackUpDirectionalAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::BackUp>(xml_tag_name, action_name, conf)
{
  double dist;
  getInput("backup_dist", dist);
  double speed;
  getInput("backup_speed", speed);
  double time_allowance;
  getInput("time_allowance", time_allowance);

  // Populate the input message
  goal_.target.x = dist;
  goal_.target.y = 0.0;
  goal_.target.z = 0.0;
  goal_.speed = speed; // The base speed is positive which will ensure regular backup
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);

  tf_buffer_ =
    config().blackboard->template get<std::shared_ptr<tf2_ros::Buffer>>(
    "tf_buffer");
}

void BackUpDirectionalAction::on_tick()
{
  nav_msgs::msg::Path path;
  getInput("truncated_path", path);

  geometry_msgs::msg::PoseStamped reference_pose = path.poses.back();
  reference_pose.header.frame_id = path.header.frame_id;
  reference_pose.header.stamp = path.header.stamp;

  geometry_msgs::msg::PoseStamped transformed_pose;

  nav2_util::transformPoseInTargetFrame(reference_pose, transformed_pose, *tf_buffer_, "base_link", 1.0);

  if (transformed_pose.pose.position.x >= 0) {
    goal_.speed = std::fabs(goal_.speed);
  } else {
    goal_.speed = -std::fabs(goal_.speed);
  }

  increment_recovery_count();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::BackUpDirectionalAction>(
        name, "backup", config);
    };

  factory.registerBuilder<nav2_behavior_tree::BackUpDirectionalAction>("BackUpDirectional", builder);
}
