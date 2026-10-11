// Copyright (c) 2026 Maurice Alexander Purnawan
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

#include <gtest/gtest.h>

#include <condition_variable>
#include <memory>
#include <mutex>

#include "nav2_ros_common/simple_action_server.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using Action = nav2_msgs::action::NavigateToPose;
using namespace std::chrono_literals;

class PendingGoalTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_node_ = std::make_shared<rclcpp::Node>("pending_goal_server");
    client_node_ = std::make_shared<rclcpp::Node>("pending_goal_client");
    server_ = std::make_shared<nav2::SimpleActionServer<Action>>(
      server_node_, "pending_goal_test", [this]() {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock, [this]() {return release_;});
        server_->succeeded_current(std::make_shared<Action::Result>());
      });
    server_->activate();
    client_ = rclcpp_action::create_client<Action>(client_node_, "pending_goal_test");
    executor_.add_node(server_node_);
    executor_.add_node(client_node_);
    ASSERT_TRUE(client_->wait_for_action_server(5s));
  }

  void TearDown() override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      release_ = true;
    }
    ready_.notify_all();
    server_->deactivate();
    executor_.remove_node(client_node_);
    executor_.remove_node(server_node_);
    client_.reset();
    server_.reset();
    client_node_.reset();
    server_node_.reset();
  }

  rclcpp_action::ClientGoalHandle<Action>::SharedPtr send(double x)
  {
    Action::Goal goal;
    goal.pose.pose.position.x = x;
    auto future = client_->async_send_goal(goal);
    if (executor_.spin_until_future_complete(future, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
      return nullptr;
    }
    return future.get();
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr server_node_;
  rclcpp::Node::SharedPtr client_node_;
  nav2::SimpleActionServer<Action>::SharedPtr server_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  std::mutex mutex_;
  std::condition_variable ready_;
  bool release_ = false;
};

TEST_F(PendingGoalTest, SupersededValidationCannotAcceptOrRejectNewerPendingGoal)
{
  ASSERT_NE(send(1.0), nullptr);
  const auto original = server_->get_current_goal();
  ASSERT_NE(send(2.0), nullptr);
  const auto validated = server_->get_pending_goal();
  ASSERT_NE(validated, nullptr);
  ASSERT_NE(send(3.0), nullptr);
  const auto newest = server_->get_pending_goal();
  ASSERT_NE(newest, validated);
  // TF validation of goal 2 can finish after goal 3 has replaced it.
  EXPECT_EQ(server_->accept_pending_goal(validated), nullptr);
  server_->terminate_pending_goal(validated);
  EXPECT_EQ(server_->get_current_goal(), original);
  EXPECT_EQ(server_->get_pending_goal(), newest);
  EXPECT_TRUE(server_->is_preempt_requested());
  EXPECT_EQ(server_->accept_pending_goal(newest), newest);
  EXPECT_EQ(server_->get_current_goal(), newest);
  EXPECT_EQ(server_->get_pending_goal(), nullptr);
  EXPECT_FALSE(server_->is_preempt_requested());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
