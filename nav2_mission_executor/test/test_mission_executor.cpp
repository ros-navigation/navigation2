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

#include <memory>

// #include "gtest/gtest.h"
#include "nav2_msgs/action/execute_mission.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

static const char xml_text[] =
  R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <NavigateToPose position="1;2;0" orientation="0;0;0;1"/>
    </Sequence>
  </BehaviorTree>
</root>
)";

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("mission_executor_test_node");
  auto action_client =
    rclcpp_action::create_client<nav2_msgs::action::ExecuteMission>(node, "ExecuteMission");

  action_client->wait_for_action_server();

  // The goal contains the XML representation of the BT
  auto goal = nav2_msgs::action::ExecuteMission::Goal();
  goal.mission_plan.mission_plan = xml_text;

  // Send the goal
  auto future_goal_handle = action_client->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(node, future_goal_handle) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "send goal call failed");
    return 1;
  }

  auto goal_handle = future_goal_handle.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    return 1;
  }

  // Wait for the result
  auto future_result = goal_handle->async_result();
  if (rclcpp::spin_until_future_complete(node, future_result) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "get result call failed");
    return 1;
  }

  auto wrapped_result = future_result.get();
  int rc = 0;

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;

    case rclcpp_action::ResultCode::ABORTED:
    case rclcpp_action::ResultCode::CANCELED:
    default:
      RCLCPP_ERROR(node->get_logger(), "Mission failed");
      rc = 1;
      break;
  }

  rclcpp::shutdown();
  return rc;
}
