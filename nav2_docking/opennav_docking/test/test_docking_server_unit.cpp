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
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "opennav_docking/docking_server.hpp"
#include "nav2_util/node_thread.hpp"

// Testing unit functions in docking server, smoke/system tests in python file

using namespace std::chrono_literals;  // NOLINT

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

namespace opennav_docking
{

class DockingServerShim : public DockingServer
{
public:
  DockingServerShim()
  : DockingServer() {}

  // Bypass TF
  virtual geometry_msgs::msg::PoseStamped getRobotPoseInFrame(const std::string &)
  {
    return geometry_msgs::msg::PoseStamped();
  }
};

TEST(DockingServerTests, ObjectLifecycle)
{
  auto node = std::make_shared<opennav_docking::DockingServer>();
  node->configure();
  node->activate();
  node->deactivate();
  node->cleanup();
  node->shutdown();
  node.reset();
}

TEST(DockingServerTests, testErrorExceptions)
{
  auto node = std::make_shared<DockingServerShim>();
  auto node_thread = nav2_util::NodeThread(node);
  auto node2 = std::make_shared<rclcpp::Node>("client_node");

  // Setup 1 instance of the test failure dock & its plugin instance
  node->declare_parameter(
    "docks",
    rclcpp::ParameterValue(std::vector<std::string>{"test_dock"}));
  node->declare_parameter(
    "test_dock.type",
    rclcpp::ParameterValue(std::string{"dock_plugin"}));
  node->declare_parameter(
    "test_dock.pose",
    rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0}));
  node->declare_parameter(
    "dock_plugins",
    rclcpp::ParameterValue(std::vector<std::string>{"dock_plugin"}));
  node->declare_parameter(
    "dock_plugin.plugin",
    rclcpp::ParameterValue(std::string{"opennav_docking::TestFailureDock"}));

  node->on_configure(rclcpp_lifecycle::State());
  node->on_activate(rclcpp_lifecycle::State());

  node->declare_parameter("exception_to_throw", rclcpp::ParameterValue(""));
  node->declare_parameter("dock_action_called", rclcpp::ParameterValue(false));

  // Error codes docking
  std::vector<std::string> error_ids{
    "TransformException", "DockNotInDB", "DockNotValid",
    "FailedToStage", "FailedToDetectDock", "FailedToControl", "FailedToCharge",
    "DockingException", "exception"};
  std::vector<int> error_codes{999, 901, 902, 903, 904, 905, 906, 999, 999};

  // Call action, check error code
  for (unsigned int i = 0; i != error_ids.size(); i++) {
    node->set_parameter(
      rclcpp::Parameter(
        "exception_to_throw",
        rclcpp::ParameterValue(error_ids[i])));

    auto client = rclcpp_action::create_client<DockRobot>(node2, "dock_robot");
    if (!client->wait_for_action_server(1s)) {
      RCLCPP_ERROR(node2->get_logger(), "Action server not available after waiting");
    }
    auto goal_msg = DockRobot::Goal();
    if (1 == 0) {
      goal_msg.use_dock_id = false;
      goal_msg.dock_type = "dock_plugin";
    } else {
      goal_msg.dock_id = "test_dock";
      goal_msg.navigate_to_staging_pose = false;
    }

    auto future_goal_handle = client->async_send_goal(goal_msg);

    if (rclcpp::spin_until_future_complete(
        node2, future_goal_handle, 2s) == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto future_result = client->async_get_result(future_goal_handle.get());
      if (rclcpp::spin_until_future_complete(
          node2, future_result, 5s) == rclcpp::FutureReturnCode::SUCCESS)
      {
        auto result = future_result.get();
        EXPECT_EQ(result.result->error_code, error_codes[i]);
      } else {
        EXPECT_TRUE(false);
      }
    } else {
      EXPECT_TRUE(false);
    }
  }

  // Now for undocking
  std::vector<std::string> error_ids_undocking{
    "TransformException", "DockNotValid", "FailedToControl",
    "DockingException", "exception"};
  std::vector<int> error_codes_undocking{999, 902, 905, 999, 999};

  // Set dock_action_called to true to simulate robot being docked in dock plugin
  node->set_parameter(rclcpp::Parameter("dock_action_called", true));

  // Call action, check error code
  for (unsigned int i = 0; i != error_ids_undocking.size(); i++) {
    node->set_parameter(
      rclcpp::Parameter(
        "exception_to_throw",
        rclcpp::ParameterValue(error_ids_undocking[i])));

    auto client = rclcpp_action::create_client<UndockRobot>(node2, "undock_robot");
    if (!client->wait_for_action_server(1s)) {
      RCLCPP_ERROR(node2->get_logger(), "Action server not available after waiting");
    }
    auto goal_msg = UndockRobot::Goal();
    goal_msg.dock_type = "dock_plugin";
    auto future_goal_handle = client->async_send_goal(goal_msg);

    if (rclcpp::spin_until_future_complete(
        node2, future_goal_handle, 2s) == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto future_result = client->async_get_result(future_goal_handle.get());
      if (rclcpp::spin_until_future_complete(
          node2, future_result, 5s) == rclcpp::FutureReturnCode::SUCCESS)
      {
        auto result = future_result.get();
        EXPECT_EQ(result.result->error_code, error_codes_undocking[i]);
      } else {
        EXPECT_TRUE(false);
      }
    } else {
      EXPECT_TRUE(false);
    }
  }
  node->on_deactivate(rclcpp_lifecycle::State());
  node->on_cleanup(rclcpp_lifecycle::State());
  node->on_shutdown(rclcpp_lifecycle::State());
  node.reset();
}

TEST(DockingServerTests, getateGoalDock)
{
  auto node = std::make_shared<opennav_docking::DockingServer>();

  // Setup 1 instance of the test failure dock & its plugin instance
  node->declare_parameter(
    "docks",
    rclcpp::ParameterValue(std::vector<std::string>{"test_dock"}));
  node->declare_parameter(
    "test_dock.type",
    rclcpp::ParameterValue(std::string{"dock_plugin"}));
  node->declare_parameter(
    "test_dock.pose",
    rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0}));
  node->declare_parameter(
    "dock_plugins",
    rclcpp::ParameterValue(std::vector<std::string>{"dock_plugin"}));
  node->declare_parameter(
    "dock_plugin.plugin",
    rclcpp::ParameterValue(std::string{"opennav_docking::TestFailureDock"}));

  node->on_configure(rclcpp_lifecycle::State());
  std::shared_ptr<const DockRobot::Goal> goal = std::make_shared<const DockRobot::Goal>();
  auto dock = node->generateGoalDock(goal);
  EXPECT_NE(dock->plugin, nullptr);
  EXPECT_EQ(dock->frame, std::string());
  node->stashDockData(false, dock, true);
  node->on_cleanup(rclcpp_lifecycle::State());
  node->on_shutdown(rclcpp_lifecycle::State());
  node.reset();
}

TEST(DockingServerTests, testDynamicParams)
{
  auto node = std::make_shared<opennav_docking::DockingServer>();

  // Setup 1 instance of the test failure dock & its plugin instance
  node->declare_parameter(
    "docks",
    rclcpp::ParameterValue(std::vector<std::string>{"test_dock"}));
  node->declare_parameter(
    "test_dock.type",
    rclcpp::ParameterValue(std::string{"dock_plugin"}));
  node->declare_parameter(
    "test_dock.pose",
    rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0}));
  node->declare_parameter(
    "dock_plugins",
    rclcpp::ParameterValue(std::vector<std::string>{"dock_plugin"}));
  node->declare_parameter(
    "dock_plugin.plugin",
    rclcpp::ParameterValue(std::string{"opennav_docking::TestFailureDock"}));

  node->on_configure(rclcpp_lifecycle::State());
  node->on_activate(rclcpp_lifecycle::State());

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("controller_frequency", 0.2),
      rclcpp::Parameter("initial_perception_timeout", 1.0),
      rclcpp::Parameter("wait_charge_timeout", 1.2),
      rclcpp::Parameter("undock_linear_tolerance", 0.25),
      rclcpp::Parameter("undock_angular_tolerance", 0.125),
      rclcpp::Parameter("base_frame", std::string("hi")),
      rclcpp::Parameter("fixed_frame", std::string("hi")),
      rclcpp::Parameter("max_retries", 7)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("controller_frequency").as_double(), 0.2);
  EXPECT_EQ(node->get_parameter("initial_perception_timeout").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("wait_charge_timeout").as_double(), 1.2);
  EXPECT_EQ(node->get_parameter("undock_linear_tolerance").as_double(), 0.25);
  EXPECT_EQ(node->get_parameter("undock_angular_tolerance").as_double(), 0.125);
  EXPECT_EQ(node->get_parameter("base_frame").as_string(), std::string("hi"));
  EXPECT_EQ(node->get_parameter("fixed_frame").as_string(), std::string("hi"));
  EXPECT_EQ(node->get_parameter("max_retries").as_int(), 7);

  node->on_deactivate(rclcpp_lifecycle::State());
  node->on_cleanup(rclcpp_lifecycle::State());
  node->on_shutdown(rclcpp_lifecycle::State());
  node.reset();
}

}  // namespace opennav_docking
