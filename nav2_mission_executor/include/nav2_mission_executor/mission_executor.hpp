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

#ifndef NAV2_MISSION_EXECUTOR__MISSION_EXECUTOR_HPP_
#define NAV2_MISSION_EXECUTOR__MISSION_EXECUTOR_HPP_

#include <string>
#include <memory>
#include "nav2_tasks/task_status.hpp"
#include "nav2_tasks/execute_mission_task.hpp"
#include "nav2_tasks/navigate_to_pose_task.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_mission_execution
{

class MissionExecutor : public nav2_tasks::ExecuteMissionTaskServer
{
public:
  MissionExecutor();
  ~MissionExecutor();

  nav2_tasks::TaskStatus execute(
    const nav2_tasks::ExecuteMissionCommand::SharedPtr command) override;

private:
  void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  std::unique_ptr<nav2_tasks::NavigateToPoseTaskClient> navTaskClient_;

  // For now, use the move_base_simple/goal topic (from rviz) for the goal pose
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_;

  // Also, for now, publish a mission plan when receiving a goal pose from rviz
  rclcpp::Publisher<nav2_msgs::msg::MissionPlan>::SharedPtr plan_pub_;
};

}  // namespace nav2_mission_execution

#endif  // NAV2_MISSION_EXECUTOR__MISSION_EXECUTOR_HPP_
