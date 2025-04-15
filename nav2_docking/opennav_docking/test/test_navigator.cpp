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

#include <functional>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "opennav_docking/navigator.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

// Test navigator

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

namespace opennav_docking
{

class DummyNavigationServer : rclcpp::Node
{
public:
  using ActionT = nav2_msgs::action::NavigateToPose;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  DummyNavigationServer()
  : Node("dummy_navigator")
  {
    action_server_ = std::make_shared<ActionServer>(
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      "navigate_to_pose", std::bind(&DummyNavigationServer::executeCallback, this),
      nullptr, std::chrono::milliseconds(500), true);

    action_server_->activate();
  }

  ~DummyNavigationServer() = default;

  void setReturn(const bool rtn)
  {
    return_state_ = rtn;
  }

  void setToggle()
  {
    toggle_ = true;
  }

  void executeCallback()
  {
    auto result = std::make_shared<typename ActionT::Result>();
    auto goal = action_server_->get_current_goal();
    (void)goal;

    bool rtn = return_state_;
    if (toggle_) {
      return_state_ = !return_state_;
    }

    if (rtn) {
      action_server_->succeeded_current(result);
      return;
    }
    action_server_->terminate_current(result);
  }

protected:
  std::shared_ptr<ActionServer> action_server_;
  bool return_state_{true};
  bool toggle_{false};
};

TEST(NavigatorTests, TestNavigatorReconfigure)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  auto navigator = std::make_unique<Navigator>(node);
  node->configure();
  node->activate();
  navigator->activate();
  navigator->deactivate();
  navigator.reset();

  // Create and activate again
  EXPECT_NO_THROW(navigator = std::make_unique<Navigator>(node));
  navigator->activate();
  navigator->deactivate();

  // Reset the node
  navigator.reset();
  node->deactivate();
  node->cleanup();
  node->shutdown();
}

TEST(NavigatorTests, TestNavigator)
{
  auto dummy_navigator_node = std::make_shared<DummyNavigationServer>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  auto navigator = std::make_unique<Navigator>(node);
  navigator->activate();

  std::function<bool()> is_preempted_true = []() {return true;};
  std::function<bool()> is_preempted_false = []() {return false;};

  // Should succeed, action server set to succeed
  dummy_navigator_node->setReturn(true);
  EXPECT_NO_THROW(
    navigator->goToPose(geometry_msgs::msg::PoseStamped(), rclcpp::Duration(10.0, 10.0),
      is_preempted_false));

  // Should fail, timeout exceeded
  EXPECT_THROW(
    navigator->goToPose(geometry_msgs::msg::PoseStamped(), rclcpp::Duration(0.0, 0.0),
      is_preempted_false),
    opennav_docking_core::FailedToStage);

  // Should fail, action server set to succeed
  dummy_navigator_node->setReturn(false);
  EXPECT_THROW(
    navigator->goToPose(geometry_msgs::msg::PoseStamped(), rclcpp::Duration(10.0, 10.0),
      is_preempted_false),
    opennav_docking_core::FailedToStage);

  // First should fail, recursion should succeed
  dummy_navigator_node->setToggle();
  EXPECT_NO_THROW(
    navigator->goToPose(geometry_msgs::msg::PoseStamped(), rclcpp::Duration(10.0, 10.0),
      is_preempted_false));

  // Should fail, preempted
  dummy_navigator_node->setReturn(true);
  EXPECT_THROW(
    navigator->goToPose(geometry_msgs::msg::PoseStamped(), rclcpp::Duration(10.0, 10.0),
      is_preempted_true),
    opennav_docking_core::FailedToStage);

  navigator->deactivate();
  navigator.reset();
  dummy_navigator_node.reset();
}

}  // namespace opennav_docking
