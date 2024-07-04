// Copyright (c) 2024 Open Navigation LLC
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

#include <chrono>
#include "opennav_docking/navigator.hpp"

namespace opennav_docking
{

using namespace std::chrono_literals;  // NOLINT

Navigator::Navigator(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent)
: node_(parent)
{
  auto node = node_.lock();
  node->declare_parameter("navigator_bt_xml", std::string(""));
  node->get_parameter("navigator_bt_xml", navigator_bt_xml_);
}

void Navigator::activate()
{
  // Need separate callback group and executor to call Nav action within docking action
  auto node = node_.lock();
  callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  executor_.add_callback_group(callback_group_, node->get_node_base_interface());
  nav_to_pose_client_ = rclcpp_action::create_client<Nav2Pose>(
    node->get_node_base_interface(),
    node->get_node_graph_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    "navigate_to_pose", callback_group_);
}

void Navigator::deactivate()
{
  nav_to_pose_client_.reset();
}

void Navigator::goToPose(
  const geometry_msgs::msg::PoseStamped & pose,
  const rclcpp::Duration & max_staging_duration,
  bool recursed)
{
  Nav2Pose::Goal goal;
  goal.pose = pose;
  goal.behavior_tree = navigator_bt_xml_;
  const auto timeout = max_staging_duration.to_chrono<std::chrono::milliseconds>();

  // Wait for server to be active
  nav_to_pose_client_->wait_for_action_server(1s);
  auto future_goal_handle = nav_to_pose_client_->async_send_goal(goal);
  if (executor_.spin_until_future_complete(
      future_goal_handle, 2s) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto future_result = nav_to_pose_client_->async_get_result(future_goal_handle.get());
    if (executor_.spin_until_future_complete(
        future_result, timeout) == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = future_result.get();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->error_code == 0) {
        return;  // Success!
      }
    }
  }

  // Attempt to retry once using single iteration recursion
  if (!recursed) {
    goToPose(pose, max_staging_duration, true);
    return;
  }

  throw opennav_docking_core::FailedToStage("Navigation request to staging pose failed.");
}

}  // namespace opennav_docking
