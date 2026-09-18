// Copyright (c) 2021, Samsung Research America
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
// limitations under the License. Reserved.

#include <math.h>

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_controller/controller_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/tf2_factories.hpp"

class ControllerServerShim : public nav2_controller::ControllerServer
{
public:
  using nav2_controller::ControllerServer::ControllerServer;

  void setEndPoseFrame(const std::string & frame) {end_pose_.header.frame_id = frame;}
  void callTransformedPlanAndGoal(
    const geometry_msgs::msg::PoseStamped & current_robot_pose)
  {
    transformedPlanAndGoal(current_robot_pose);
  }
  geometry_msgs::msg::PoseStamped callGetCurrentRobotPose() {return getCurrentRobotPose();}
  nav2::TransformBuffer & getTfBuffer() {return *costmap_ros_->getTfBuffer();}
};

TEST(ControllerServerTest, TransformedPlanAndGoalThrowsOnTfFailure)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("progress_checker_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("goal_checker_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("controller_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("path_handler_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("filters", std::vector<std::string>{}),
  });

  auto server = std::make_shared<ControllerServerShim>(options);
  ASSERT_EQ(
    server->configure().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = server->now();
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "base_link";
  tf_msg.transform.rotation.w = 1.0;
  server->getTfBuffer().setTransform(tf_msg, "test", true);

  geometry_msgs::msg::PoseStamped current_robot_pose;
  current_robot_pose.header.stamp = server->now();
  current_robot_pose.header.frame_id = "map";
  current_robot_pose.pose.orientation.w = 1.0;

  server->setEndPoseFrame("nonexistent_frame_xyz");
  EXPECT_THROW(
    server->callTransformedPlanAndGoal(current_robot_pose), nav2_core::ControllerTFError);
  server->cleanup();
}

TEST(ControllerServerTest, RejectsStaleRobotPoseTransform)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("transform_staleness_threshold", 0.1),
    rclcpp::Parameter("progress_checker_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("goal_checker_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("controller_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("path_handler_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("filters", std::vector<std::string>{}),
  });

  auto server = std::make_shared<ControllerServerShim>(options);
  ASSERT_EQ(
    server->configure().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = server->now() - rclcpp::Duration::from_seconds(1.0);
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "base_link";
  tf_msg.transform.rotation.w = 1.0;
  server->getTfBuffer().setTransform(tf_msg, "test", false);

  try {
    server->callGetCurrentRobotPose();
    FAIL() << "Expected stale robot pose transform to throw";
  } catch (const nav2_core::ControllerTFError & exception) {
    const std::string error_message = exception.what();
    EXPECT_NE(error_message.find("base_link"), std::string::npos);
    EXPECT_NE(error_message.find("map"), std::string::npos);
  }

  tf_msg.header.stamp = server->now();
  server->getTfBuffer().setTransform(tf_msg, "test", false);
  EXPECT_NO_THROW(server->callGetCurrentRobotPose());
  server->cleanup();
}

TEST(ControllerServerTest, SkipsRobotPoseTransformStalenessCheckWhenDisabled)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("transform_staleness_threshold", -1.0),
    rclcpp::Parameter("progress_checker_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("goal_checker_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("controller_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("path_handler_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("filters", std::vector<std::string>{}),
  });

  auto server = std::make_shared<ControllerServerShim>(options);
  ASSERT_EQ(
    server->configure().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = server->now() - rclcpp::Duration::from_seconds(1.0);
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "base_link";
  tf_msg.transform.rotation.w = 1.0;
  server->getTfBuffer().setTransform(tf_msg, "test", false);

  EXPECT_NO_THROW(server->callGetCurrentRobotPose());
  server->cleanup();
}

TEST(ControllerServerTest, GoalCheckerPluginTypeException)
{
  std::string nodeName = "test_node";
  auto node = std::make_shared<nav2::LifecycleNode>(nodeName);
  std::vector<std::string> invalid_ids = {"invalid_goal_checker"};
  node->declare_parameter("goal_checker_plugins", invalid_ids);
  node->declare_parameter("invalid_goal_checker.plugin", rclcpp::PARAMETER_STRING);
  EXPECT_THROW(std::make_unique<nav2_controller::ParameterHandler>(node, node->get_logger()),
    std::runtime_error);
}

TEST(ControllerServerTest, ProgressCheckerPluginTypeException)
{
  std::string nodeName = "test_node";
  auto node = std::make_shared<nav2::LifecycleNode>(nodeName);
  std::vector<std::string> invalid_ids = {"invalid_progress_checker"};
  node->declare_parameter("progress_checker_plugins", invalid_ids);
  node->declare_parameter("invalid_progress_checker.plugin", rclcpp::PARAMETER_STRING);
  EXPECT_THROW(std::make_unique<nav2_controller::ParameterHandler>(node, node->get_logger()),
    std::runtime_error);
}

TEST(ControllerServerTest, ControllerPluginTypeException)
{
  std::string nodeName = "test_node";
  auto node = std::make_shared<nav2::LifecycleNode>(nodeName);
  std::vector<std::string> invalid_ids = {"invalid_controller"};
  node->declare_parameter("controller_plugins", invalid_ids);
  node->declare_parameter("invalid_controller.plugin", rclcpp::PARAMETER_STRING);
  EXPECT_THROW(std::make_unique<nav2_controller::ParameterHandler>(node, node->get_logger()),
    std::runtime_error);
}

TEST(ControllerServerTest, PathHandlerTypeException)
{
  std::string nodeName = "test_node";
  auto node = std::make_shared<nav2::LifecycleNode>(nodeName);
  std::vector<std::string> invalid_ids = {"invalid_path_handler"};
  node->declare_parameter("path_handler_plugins", invalid_ids);
  node->declare_parameter("invalid_path_handler.plugin", rclcpp::PARAMETER_STRING);
  EXPECT_THROW(std::make_unique<nav2_controller::ParameterHandler>(node, node->get_logger()),
    std::runtime_error);
}

TEST(ControllerServerTest, test_dynamic_parameters)
{
  std::string nodeName = "test_node";
  auto node = std::make_shared<nav2::LifecycleNode>(nodeName);
  auto param_handler_ = std::make_unique<nav2_controller::ParameterHandler>(
      node, node->get_logger());
  param_handler_->activate();
  auto params_ = param_handler_->getParams();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("min_x_velocity_threshold", 100.0),
      rclcpp::Parameter("min_y_velocity_threshold", 100.0),
      rclcpp::Parameter("min_theta_velocity_threshold", 100.0),
      rclcpp::Parameter("failure_tolerance", 5.0),
      rclcpp::Parameter("search_window", 10.0),
      rclcpp::Parameter("transform_staleness_threshold", 1.0)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(params_->min_x_velocity_threshold, 100.0);
  EXPECT_EQ(params_->min_y_velocity_threshold, 100.0);
  EXPECT_EQ(params_->min_theta_velocity_threshold, 100.0);
  EXPECT_EQ(params_->failure_tolerance, 5.0);
  EXPECT_EQ(params_->search_window, 10.0);
  EXPECT_EQ(params_->transform_staleness_threshold, 1.0);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("min_x_velocity_threshold", -1.0)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(params_->min_x_velocity_threshold, 100.0);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("search_window", -0.1)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(params_->search_window, 10.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
