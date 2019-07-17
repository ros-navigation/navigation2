// Copyright (c) 2019 Intel Corporation
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

#include "nav2_rviz_plugins/goal_tool.hpp"

#include <memory>
#include <string>

#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/properties/string_property.hpp"

namespace nav2_rviz_plugins
{

GoalTool::GoalTool()
: rviz_default_plugins::tools::PoseTool()
{
  shortcut_key_ = 'g';

  topic_property_ = std::make_unique<rviz_common::properties::StringProperty>("Topic", "goalpose",
      "The topic on which to publish goal poses.",
      getPropertyContainer(), SLOT(updateTopic()), this);
}

GoalTool::~GoalTool()
{
}

void GoalTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Navigation2 Goal");
  setIcon(rviz_common::loadPixmap("package://nav2_rviz_plugins/icons/SetGoal.png"));
  updateTopic();
}

void GoalTool::updateTopic()
{
  publisher_ = context_->getRosNodeAbstraction().lock()->get_raw_node()->
    template create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_property_->getStdString(), rclcpp::SystemDefaultsQoS());
}

void
GoalTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = fixed_frame;
  pose.header.stamp = rclcpp::Clock().now();

  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  pose.pose.orientation = orientationAroundZAxis(theta);

  logPose(pose.pose.position, pose.pose.orientation, theta, fixed_frame);

  publisher_->publish(pose);
}

}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::GoalTool, rviz_common::Tool)
