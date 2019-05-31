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

#include <memory>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/execute_mission.hpp"
#include "nav2_util/simple_action_server.hpp"

namespace nav2_mission_executor
{

class MissionExecutor : public nav2_util::LifecycleNode
{
public:
  MissionExecutor();
  ~MissionExecutor();

protected:
  // Implement the lifecycle interface
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  using GoalHandle = rclcpp_action::ServerGoalHandle<nav2_msgs::action::ExecuteMission>;
  using ActionServer = nav2_util::SimpleActionServer<nav2_msgs::action::ExecuteMission>;

  // Out action server implements the ExecuteMission action
  std::unique_ptr<ActionServer> action_server_;

  // The action server callback
  void executeMission(const std::shared_ptr<GoalHandle> goal_handle);

  // A regular, non-spinning ROS node that we can use for the Behavior Tree
  rclcpp::Node::SharedPtr client_node_;
};

}  // namespace nav2_mission_executor

#endif  // NAV2_MISSION_EXECUTOR__MISSION_EXECUTOR_HPP_
