// Copyright (c) 2020 ymd-stella
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
#include <vector>
#include <memory>

#include "gtest/gtest.h"
#include "nav2_bt_waypoint_follower/bt_waypoint_follower.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

TEST(follow_waypoints_action, valid_action)
{
  auto node = std::make_shared<nav2_bt_waypoint_follower::BtWaypointFollower>(false);
  std::string package_share_directory = ament_index_cpp::get_package_share_directory(
    "nav2_bt_waypoint_follower");
  node->set_parameter(
    rclcpp::Parameter(
      "bt_xml_filename",
      package_share_directory + "/test/behavior_trees/unit_test_follow_waypoints.xml"));
  nav2_util::CallbackReturn ret;
  node->configure(ret);
  EXPECT_EQ(ret, nav2_util::CallbackReturn::SUCCESS);
  node->activate(ret);
  EXPECT_EQ(ret, nav2_util::CallbackReturn::SUCCESS);

  auto client = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    node,
    "follow_waypoints");
  while (!client->wait_for_action_server(1s)) {}

  nav2_msgs::action::FollowWaypoints::Goal goal;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 1.0;
  pose.pose.orientation.w = 1.0;
  goal.poses.push_back(pose);
  goal.poses.push_back(pose);
  goal.poses.push_back(pose);
  auto goal_handle_future = client->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node, goal_handle_future);
  auto goal_handle = goal_handle_future.get();

  auto result_future = client->async_get_result(goal_handle);
  rclcpp::spin_until_future_complete(node, result_future);

  auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST(follow_waypoints_action, invalid_xml)
{
  auto node = std::make_shared<nav2_bt_waypoint_follower::BtWaypointFollower>(false);
  std::string package_share_directory = ament_index_cpp::get_package_share_directory(
    "nav2_bt_waypoint_follower");
  node->set_parameter(rclcpp::Parameter("bt_xml_filename", "not_found.xml"));
  nav2_util::CallbackReturn ret;
  node->configure(ret);
  EXPECT_EQ(ret, nav2_util::CallbackReturn::FAILURE);
}

TEST(follow_waypoints_action, no_pose)
{
  auto node = std::make_shared<nav2_bt_waypoint_follower::BtWaypointFollower>();
  std::string package_share_directory = ament_index_cpp::get_package_share_directory(
    "nav2_bt_waypoint_follower");
  node->set_parameter(
    rclcpp::Parameter(
      "bt_xml_filename",
      package_share_directory + "/test/behavior_trees/unit_test_follow_waypoints.xml"));
  nav2_util::CallbackReturn ret;
  node->configure(ret);
  EXPECT_EQ(ret, nav2_util::CallbackReturn::SUCCESS);
  node->activate(ret);
  EXPECT_EQ(ret, nav2_util::CallbackReturn::SUCCESS);

  auto client = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    node,
    "follow_waypoints");
  while (!client->wait_for_action_server(1s)) {}

  nav2_msgs::action::FollowWaypoints::Goal goal;
  auto goal_handle_future = client->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node, goal_handle_future);
  auto goal_handle = goal_handle_future.get();

  auto result_future = client->async_get_result(goal_handle);
  rclcpp::spin_until_future_complete(node, result_future);

  auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
}

TEST(follow_waypoints_action, cancel)
{
  auto node = std::make_shared<nav2_bt_waypoint_follower::BtWaypointFollower>(false);
  std::string package_share_directory = ament_index_cpp::get_package_share_directory(
    "nav2_bt_waypoint_follower");
  node->set_parameter(
    rclcpp::Parameter(
      "bt_xml_filename",
      package_share_directory + "/test/behavior_trees/unit_test_follow_waypoints_running.xml"));
  nav2_util::CallbackReturn ret;
  node->configure(ret);
  EXPECT_EQ(ret, nav2_util::CallbackReturn::SUCCESS);
  node->activate(ret);
  EXPECT_EQ(ret, nav2_util::CallbackReturn::SUCCESS);

  auto client = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    node,
    "follow_waypoints");
  while (!client->wait_for_action_server(1s)) {}

  nav2_msgs::action::FollowWaypoints::Goal goal;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 1.0;
  pose.pose.orientation.w = 1.0;
  goal.poses.push_back(pose);
  goal.poses.push_back(pose);
  goal.poses.push_back(pose);
  auto goal_handle_future = client->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node, goal_handle_future);
  auto goal_handle = goal_handle_future.get();
  auto result_future = client->async_get_result(goal_handle);

  auto cancel_future = client->async_cancel_goal(goal_handle);
  rclcpp::spin_until_future_complete(node, cancel_future);
  auto cancel_response = cancel_future.get();

  rclcpp::spin_until_future_complete(node, result_future);

  auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
}

TEST(follow_waypoints_action, preempt)
{
  auto node = std::make_shared<nav2_bt_waypoint_follower::BtWaypointFollower>(false);
  std::string package_share_directory = ament_index_cpp::get_package_share_directory(
    "nav2_bt_waypoint_follower");
  node->set_parameter(
    rclcpp::Parameter(
      "bt_xml_filename",
      package_share_directory + "/test/behavior_trees/unit_test_follow_waypoints_running.xml"));
  nav2_util::CallbackReturn ret;
  node->configure(ret);
  EXPECT_EQ(ret, nav2_util::CallbackReturn::SUCCESS);
  node->activate(ret);
  EXPECT_EQ(ret, nav2_util::CallbackReturn::SUCCESS);

  auto client = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    node,
    "follow_waypoints");
  while (!client->wait_for_action_server(1s)) {}

  nav2_msgs::action::FollowWaypoints::Goal goal;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 1.0;
  pose.pose.orientation.w = 1.0;
  goal.poses.push_back(pose);
  goal.poses.push_back(pose);
  goal.poses.push_back(pose);
  auto goal_handle_future = client->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node, goal_handle_future);
  auto goal_handle = goal_handle_future.get();

  auto goal_handle_future2 = client->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node, goal_handle_future2);
  auto goal_handle2 = goal_handle_future2.get();
  auto result_future = client->async_get_result(goal_handle2);

  auto cancel_future = client->async_cancel_goal(goal_handle2);
  rclcpp::spin_until_future_complete(node, cancel_future);
  auto cancel_response = cancel_future.get();

  rclcpp::spin_until_future_complete(node, result_future);

  auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
}

TEST(follow_waypoints_action, failure)
{
  auto node = std::make_shared<nav2_bt_waypoint_follower::BtWaypointFollower>(false);
  std::string package_share_directory = ament_index_cpp::get_package_share_directory(
    "nav2_bt_waypoint_follower");
  node->set_parameter(
    rclcpp::Parameter(
      "bt_xml_filename",
      package_share_directory + "/test/behavior_trees/unit_test_follow_waypoints_failure.xml"));
  nav2_util::CallbackReturn ret;
  node->configure(ret);
  EXPECT_EQ(ret, nav2_util::CallbackReturn::SUCCESS);
  node->activate(ret);
  EXPECT_EQ(ret, nav2_util::CallbackReturn::SUCCESS);

  auto client = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    node,
    "follow_waypoints");
  while (!client->wait_for_action_server(1s)) {}

  nav2_msgs::action::FollowWaypoints::Goal goal;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 1.0;
  pose.pose.orientation.w = 1.0;
  goal.poses.push_back(pose);
  goal.poses.push_back(pose);
  goal.poses.push_back(pose);
  auto goal_handle_future = client->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node, goal_handle_future);
  auto goal_handle = goal_handle_future.get();

  auto result_future = client->async_get_result(goal_handle);
  rclcpp::spin_until_future_complete(node, result_future);

  auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
