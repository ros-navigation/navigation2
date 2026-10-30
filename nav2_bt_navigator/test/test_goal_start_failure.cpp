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
#include <unistd.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "nav2_bt_navigator/navigators/navigate_to_pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using Action = nav2_msgs::action::NavigateToPose;
using namespace std::chrono_literals;

enum class Failure {FreshStart, PromotedTf, PromotedMuxer};

class OtherNavigator : public nav2_core::NavigatorBase
{
public:
  bool on_configure(
    nav2::LifecycleNode::WeakPtr,
    const std::vector<std::string> &,
    const nav2_core::FeedbackUtils &,
    nav2_core::NavigatorMuxer *,
    std::shared_ptr<nav2_util::OdomSmoother>) override {return true;}
  bool on_activate() override {return true;}
  bool on_deactivate() override {return true;}
  bool on_cleanup() override {return true;}
  void preempt() override {}
};

// Injects each failure at the moment it happens, on top of the real navigator callbacks.
class FailingNavigator : public nav2_bt_navigator::NavigateToPoseNavigator
{
public:
  Failure failure{Failure::FreshStart};
  nav2_core::NavigatorMuxer * muxer{nullptr};
  OtherNavigator * other{nullptr};
  std::shared_ptr<nav2::TransformBuffer> tf_buffer;
  std::atomic<int> received{0};
  std::atomic<int> started{0};

protected:
  bool goalReceived(Action::Goal::ConstSharedPtr goal) override
  {
    ++received;
    return NavigateToPoseNavigator::goalReceived(goal);
  }

  bool goalStarted(Action::Goal::ConstSharedPtr goal) override
  {
    const int count = ++started;
    if (failure == Failure::FreshStart || (failure == Failure::PromotedTf && count == 2)) {
      tf_buffer->clear();
    }
    return NavigateToPoseNavigator::goalStarted(goal);
  }

  // Leave the second goal pending so that it is promoted after the first one completes.
  void onPreempt(Action::Goal::ConstSharedPtr) override {}

  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    nav2_behavior_tree::BtStatus & status) override
  {
    NavigateToPoseNavigator::goalCompleted(result, status);
    if (failure == Failure::PromotedMuxer && !muxer->isNavigating() && started == 1) {
      muxer->startNavigating(other);
    }
  }
};

class GoalStartFailure : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // A delay keeps the first goal running while the second one is sent.
    const std::string bt_dir = "/tmp/goal_start_failure_test_" + std::to_string(getpid());
    const std::string bt_xml = bt_dir + "/tree.xml";
    std::filesystem::create_directories(bt_dir);
    std::ofstream(bt_xml) <<
      "<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">"
      "<BehaviorTree ID=\"MainTree\"><Delay delay_msec=\"1000\"><AlwaysSuccess/></Delay>"
      "</BehaviorTree></root>";
    node_ = std::make_shared<nav2::LifecycleNode>("goal_start_failure_test");
    node_->declare_parameter("default_nav_to_pose_bt_xml", bt_xml);
    node_->declare_parameter("bt_search_directories", std::vector<std::string>{bt_dir});
    tf_ = std::make_shared<nav2::TransformBuffer>(node_->get_clock());
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf_->setTransform(transform, "test", true));
    nav_.muxer = &muxer_;
    nav_.other = &other_;
    nav_.tf_buffer = tf_;
    ASSERT_TRUE(nav_.on_configure(node_, {}, {"base_link", "map", 0.0, tf_}, &muxer_, nullptr));
    ASSERT_TRUE(nav_.on_activate());

    client_node_ = std::make_shared<rclcpp::Node>("goal_start_failure_client");
    client_ = rclcpp_action::create_client<Action>(client_node_, "navigate_to_pose");
    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(client_node_);
    ASSERT_TRUE(client_->wait_for_action_server(5s));
  }

  void TearDown() override
  {
    nav_.on_deactivate();
    nav_.on_cleanup();
  }

  rclcpp_action::ClientGoalHandle<Action>::SharedPtr send()
  {
    Action::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.pose.orientation.w = 1.0;
    auto future = client_->async_send_goal(goal);
    EXPECT_EQ(executor_.spin_until_future_complete(future, 5s), rclcpp::FutureReturnCode::SUCCESS);
    return future.get();
  }

  rclcpp_action::ClientGoalHandle<Action>::WrappedResult result(
    rclcpp_action::ClientGoalHandle<Action>::SharedPtr handle)
  {
    auto future = client_->async_get_result(handle);
    EXPECT_EQ(
      executor_.spin_until_future_complete(future, 10s), rclcpp::FutureReturnCode::SUCCESS);
    return future.get();
  }

  // The first goal runs to completion, then the pending second goal fails to start.
  rclcpp_action::ClientGoalHandle<Action>::WrappedResult runPromotedGoal()
  {
    EXPECT_NE(send(), nullptr);
    executor_.spin_all(200ms);
    auto pending = send();
    EXPECT_NE(pending, nullptr);
    return result(pending);
  }

  nav2_core::NavigatorMuxer muxer_;
  OtherNavigator other_;
  FailingNavigator nav_;
  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2::TransformBuffer> tf_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(GoalStartFailure, FreshGoalReportsTfErrorAndReleasesMuxer)
{
  nav_.failure = Failure::FreshStart;
  auto handle = send();
  ASSERT_NE(handle, nullptr);
  const auto res = result(handle);

  EXPECT_EQ(res.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(res.result->error_code, Action::Result::TF_ERROR);
  EXPECT_FALSE(muxer_.isNavigating());
}

TEST_F(GoalStartFailure, PromotedGoalReportsTfErrorAndReleasesMuxer)
{
  nav_.failure = Failure::PromotedTf;
  const auto res = runPromotedGoal();

  EXPECT_EQ(res.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(res.result->error_code, Action::Result::TF_ERROR);
  EXPECT_EQ(nav_.received, 2);
  EXPECT_EQ(nav_.started, 2);
  EXPECT_FALSE(muxer_.isNavigating());
}

TEST_F(GoalStartFailure, PromotedGoalRejectedWhileOtherNavigatorOwnsMuxer)
{
  nav_.failure = Failure::PromotedMuxer;
  const auto res = runPromotedGoal();

  EXPECT_EQ(res.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(res.result->error_code, Action::Result::GOAL_REJECTED);
  EXPECT_EQ(nav_.started, 1);
  // The rejected goal must not take or release the other navigator's ownership.
  EXPECT_TRUE(muxer_.isCurrentNavigator(&other_));
  muxer_.stopNavigating(&other_);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
