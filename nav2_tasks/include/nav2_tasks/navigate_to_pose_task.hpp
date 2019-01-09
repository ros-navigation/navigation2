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

#ifndef NAV2_TASKS__NAVIGATE_TO_POSE_TASK_HPP_
#define NAV2_TASKS__NAVIGATE_TO_POSE_TASK_HPP_

#include <memory>
#include "nav2_tasks/task_client.hpp"
#include "nav2_tasks/task_server.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/empty.hpp"

namespace nav2_tasks
{

using NavigateToPoseCommand = geometry_msgs::msg::PoseStamped;
using NavigateToPoseResult = std_msgs::msg::Empty;

using NavigateToPoseTaskClient = TaskClient<NavigateToPoseCommand, NavigateToPoseResult>;

class NavigateToPoseTaskServer : public TaskServer<NavigateToPoseCommand, NavigateToPoseResult>
{
public:
  explicit NavigateToPoseTaskServer(rclcpp::Node::SharedPtr & node)
  : TaskServer<NavigateToPoseCommand, NavigateToPoseResult>(node)
  {
    // A subscription to the goal pose from rviz2
    goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("move_base_simple/goal",
        std::bind(&NavigateToPoseTaskServer::onGoalPoseReceived, this, std::placeholders::_1));

    initial_pose_received_ = false;
    initial_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose",
      std::bind(&NavigateToPoseTaskServer::onInitialPoseReceived, this, std::placeholders::_1));

    // A client that we'll use to send a command message to our own task server
    self_client_ = std::make_unique<nav2_tasks::NavigateToPoseTaskClient>(node_);
  }

  NavigateToPoseTaskServer() = delete;

  bool isInitialPoseReceieved()
  {
    if (initial_pose_received_) {
      return true;
    }
    return false;
  }

  void setInitialPose(bool initial_pose)
  {
    initial_pose_received_ = initial_pose;
  }

protected:
  // For backwards compatibility, the NavigateToPoseTaskServer will respond to the goal_pose
  // message sent from rviz. We'll receive the incoming message and invoke our own
  // NavigateToPose task using self_client_
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  std::unique_ptr<nav2_tasks::NavigateToPoseTaskClient> self_client_;
  bool initial_pose_received_;
  void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
  {
    self_client_->sendCommand(pose);
  }
  void onInitialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr /*msg*/)
  {
    initial_pose_received_ = true;
  }
};

template<>
inline const char * getTaskName<NavigateToPoseCommand, NavigateToPoseResult>()
{
  return "NavigateToPoseTask";
}

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__NAVIGATE_TO_POSE_TASK_HPP_
