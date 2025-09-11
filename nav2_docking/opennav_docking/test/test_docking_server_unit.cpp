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
#include "nav2_ros_common/node_thread.hpp"

// Testing unit functions in docking server, smoke/system tests in python file

using namespace std::chrono_literals;  // NOLINT

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

  std::optional<bool> getDockBackward()
  {
    return dock_backwards_;
  }
};

// Test shim to expose the TF2 buffer for injection
class DockingServerTFShim : public DockingServerShim
{
public:
  DockingServerTFShim()
  : DockingServerShim() {}
  std::shared_ptr<tf2_ros::Buffer> getTfBuffer() {return tf2_buffer_;}
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
  auto node_thread = nav2::NodeThread(node);
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
      rclcpp::Parameter("max_retries", 7),
      rclcpp::Parameter("rotation_angular_tolerance", 0.42)});

  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  EXPECT_EQ(node->get_parameter("initial_perception_timeout").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("wait_charge_timeout").as_double(), 1.2);
  EXPECT_EQ(node->get_parameter("undock_linear_tolerance").as_double(), 0.25);
  EXPECT_EQ(node->get_parameter("undock_angular_tolerance").as_double(), 0.125);
  EXPECT_EQ(node->get_parameter("base_frame").as_string(), std::string("hi"));
  EXPECT_EQ(node->get_parameter("fixed_frame").as_string(), std::string("hi"));
  EXPECT_EQ(node->get_parameter("max_retries").as_int(), 7);
  EXPECT_EQ(node->get_parameter("rotation_angular_tolerance").as_double(), 0.42);

  node->on_deactivate(rclcpp_lifecycle::State());
  node->on_cleanup(rclcpp_lifecycle::State());
  node->on_shutdown(rclcpp_lifecycle::State());
  node.reset();
}

TEST(DockingServerTests, testDockBackward)
{
  auto node = std::make_shared<DockingServerShim>();

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

  // The dock_backwards parameter should be declared but not set
  node->on_configure(rclcpp_lifecycle::State());
  EXPECT_FALSE(node->getDockBackward().has_value());
  node->on_cleanup(rclcpp_lifecycle::State());

  // Now, set the dock_backwards parameter to true
  node->set_parameter(rclcpp::Parameter("dock_backwards", rclcpp::ParameterValue(true)));
  node->on_configure(rclcpp_lifecycle::State());
  EXPECT_TRUE(node->getDockBackward().has_value());
  EXPECT_TRUE(node->getDockBackward().value());
  node->on_cleanup(rclcpp_lifecycle::State());

  // Now, set the dock_backwards parameter to false
  node->set_parameter(rclcpp::Parameter("dock_backwards", rclcpp::ParameterValue(false)));
  node->on_configure(rclcpp_lifecycle::State());
  EXPECT_TRUE(node->getDockBackward().has_value());
  EXPECT_FALSE(node->getDockBackward().value());
  node->on_cleanup(rclcpp_lifecycle::State());

  node->on_shutdown(rclcpp_lifecycle::State());
  node.reset();
}

TEST(DockingServerTests, ExceptionHandlingDuringDocking)
{
  auto node = std::make_shared<DockingServerShim>();
  auto node_thread = nav2::NodeThread(node);
  auto client_node = std::make_shared<rclcpp::Node>("test_client");

  // Configure docking server
  node->declare_parameter("docks", std::vector<std::string>{"test_dock"});
  node->declare_parameter("test_dock.type", "test_plugin");
  node->declare_parameter("test_dock.pose", std::vector<double>{0.0, 0.0, 0.0});
  node->declare_parameter("dock_plugins", std::vector<std::string>{"test_plugin"});
  node->declare_parameter("test_plugin.plugin", "opennav_docking::TestFailureDock");
  node->declare_parameter("exception_to_throw", "");
  node->declare_parameter("dock_action_called", false);

  node->on_configure(rclcpp_lifecycle::State());
  node->on_activate(rclcpp_lifecycle::State());

  // Test multiple exception scenarios to ensure coverage
  std::vector<std::string> exceptions = {"FailedToDetectDock", "FailedToControl"};

  for (const auto & exception_type : exceptions) {
    node->set_parameter(rclcpp::Parameter("exception_to_throw", exception_type));
    node->set_parameter(rclcpp::Parameter("dock_action_called", false));

    auto client = rclcpp_action::create_client<DockRobot>(client_node, "dock_robot");
    ASSERT_TRUE(client->wait_for_action_server(2s));

    auto goal = DockRobot::Goal();
    goal.dock_id = "test_dock";
    goal.navigate_to_staging_pose = false;

    auto future_goal = client->async_send_goal(goal);
    rclcpp::spin_until_future_complete(client_node, future_goal, 2s);

    auto goal_handle = future_goal.get();
    ASSERT_TRUE(goal_handle);

    auto future_result = client->async_get_result(goal_handle);
    auto status = rclcpp::spin_until_future_complete(client_node, future_result, 5s);
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

    auto result = future_result.get();
    EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  }

  node->on_deactivate(rclcpp_lifecycle::State());
  node->on_cleanup(rclcpp_lifecycle::State());
}

TEST(DockingServerTests, StopDetectionOnSuccess)
{
  auto node = std::make_shared<DockingServerShim>();
  auto node_thread = nav2::NodeThread(node);
  auto client_node = std::make_shared<rclcpp::Node>("test_client_success");

  // Configure the server with the test plugin
  node->declare_parameter("docks", std::vector<std::string>{"test_dock"});
  node->declare_parameter("test_dock.type", "test_plugin");
  node->declare_parameter("test_dock.pose", std::vector<double>{0.0, 0.0, 0.0});
  node->declare_parameter("dock_plugins", std::vector<std::string>{"test_plugin"});
  node->declare_parameter("test_plugin.plugin", "opennav_docking::TestFailureDock");

  // Configure TestFailureDock to report success
  node->declare_parameter("exception_to_throw", "");
  node->declare_parameter("dock_action_called", true);
  // Note: isCharging() in TestFailureDock returns false, so it will wait for charge
  // which will succeed because the plugin is a charger. We'll set the timeout low.
  node->set_parameter(rclcpp::Parameter("wait_charge_timeout", 0.1));

  node->on_configure(rclcpp_lifecycle::State());
  node->on_activate(rclcpp_lifecycle::State());

  auto client = rclcpp_action::create_client<DockRobot>(client_node, "dock_robot");
  ASSERT_TRUE(client->wait_for_action_server(2s));

  DockRobot::Goal goal;
  goal.dock_id = "test_dock";
  goal.navigate_to_staging_pose = false;

  auto future_goal = client->async_send_goal(goal);
  rclcpp::spin_until_future_complete(client_node, future_goal, 2s);
  auto goal_handle = future_goal.get();
  ASSERT_TRUE(goal_handle);

  auto future_result = client->async_get_result(goal_handle);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(client_node, future_result, 5s),
    rclcpp::FutureReturnCode::SUCCESS);

  auto result = future_result.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result.result->success);

  node->on_deactivate(rclcpp_lifecycle::State());
  node->on_cleanup(rclcpp_lifecycle::State());
  node->shutdown();
}

TEST(DockingServerTests, HandlesPluginStartFailure)
{
  auto node = std::make_shared<DockingServerTFShim>();
  auto node_thread = nav2::NodeThread(node);
  auto client_node = std::make_shared<rclcpp::Node>("test_client_start_failure");

  // Configure the server with the TestFailureDock plugin.
  node->declare_parameter("docks", std::vector<std::string>{"test_dock"});
  node->declare_parameter("test_dock.type", "test_plugin");
  node->declare_parameter("test_dock.pose", std::vector<double>{0.0, 0.0, 0.0});
  node->declare_parameter("test_dock.frame", "odom");
  node->declare_parameter("dock_plugins", std::vector<std::string>{"test_plugin"});
  node->declare_parameter("test_plugin.plugin", "opennav_docking::TestFailureDock");
  node->declare_parameter("exception_to_throw", "");

  // Configure the TestFailureDock to fail its startup process.
  node->declare_parameter("fail_start_detection", true);
  node->declare_parameter("dock_action_called", false);

  node->on_configure(rclcpp_lifecycle::State());

  // Mock the necessary TF transform to prevent a premature failure.
  geometry_msgs::msg::TransformStamped identity_transform;
  identity_transform.header.frame_id = "odom";
  identity_transform.child_frame_id = "odom";
  identity_transform.transform.rotation.w = 1.0;
  node->getTfBuffer()->setTransform(identity_transform, "test_authority", true);

  node->on_activate(rclcpp_lifecycle::State());

  auto client = rclcpp_action::create_client<DockRobot>(client_node, "dock_robot");
  ASSERT_TRUE(client->wait_for_action_server(2s));

  DockRobot::Goal goal;
  goal.dock_id = "test_dock";
  goal.navigate_to_staging_pose = false;

  auto future_goal = client->async_send_goal(goal);
  rclcpp::spin_until_future_complete(client_node, future_goal, 2s);
  auto goal_handle = future_goal.get();
  ASSERT_TRUE(goal_handle);

  auto future_result = client->async_get_result(goal_handle);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(client_node, future_result, 5s),
    rclcpp::FutureReturnCode::SUCCESS);

  auto result = future_result.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.result->error_code, DockRobot::Result::FAILED_TO_DETECT_DOCK);

  node->on_deactivate(rclcpp_lifecycle::State());
  node->on_cleanup(rclcpp_lifecycle::State());
  node->shutdown();
}

}  // namespace opennav_docking

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
