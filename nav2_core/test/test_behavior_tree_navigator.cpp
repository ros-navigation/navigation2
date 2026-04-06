// Copyright (c) 2026 Botronics
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

#include <thread>

#include "nav2_core/behavior_tree_navigator.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;

// BehaviorTreeNavigator that exposes onGoalReceived() for testing.
// on_configure() is intentionally never called so no BtActionServer is created.
class TestBTNavigator : public nav2_core::BehaviorTreeNavigator<NavigateToPose>
{
public:
  void setup(
    nav2_core::NavigatorMuxer * muxer,
    bool allow_preemption,
    int timeout_ms = 500)
  {
    plugin_muxer_ = muxer;
    allow_navigator_preemption_ = allow_preemption;
    navigator_preemption_timeout_ = std::chrono::milliseconds(timeout_ms);
    logger_ = rclcpp::get_logger("test_bt_navigator");
  }

  bool callOnGoalReceived(NavigateToPose::Goal::ConstSharedPtr goal) {return onGoalReceived(goal);}

  std::string getName() override {return "test_navigator";}
  std::string getDefaultBTFilepath(nav2::LifecycleNode::WeakPtr) override {return "";}
  bool goalReceived(NavigateToPose::Goal::ConstSharedPtr) override {return goal_accepted_;}
  void onLoop() override {}
  void onPreempt(NavigateToPose::Goal::ConstSharedPtr) override {}
  void goalCompleted(
    NavigateToPose::Result::SharedPtr,
    const nav2_behavior_tree::BtStatus) override {}

  bool goal_accepted_ = true;
};

TEST(BTNavigatorOnGoalReceivedTest, NotNavigatingGoalAccepted)
{
  nav2_core::NavigatorMuxer muxer;
  TestBTNavigator nav;
  nav.setup(&muxer, false);

  auto goal = std::make_shared<NavigateToPose::Goal>();
  EXPECT_TRUE(nav.callOnGoalReceived(goal));
  EXPECT_TRUE(muxer.isNavigating());
}

TEST(BTNavigatorOnGoalReceivedTest, NotNavigatingGoalRejected)
{
  nav2_core::NavigatorMuxer muxer;
  TestBTNavigator nav;
  nav.setup(&muxer, false);
  nav.goal_accepted_ = false;

  auto goal = std::make_shared<NavigateToPose::Goal>();
  EXPECT_FALSE(nav.callOnGoalReceived(goal));
  EXPECT_FALSE(muxer.isNavigating());
}

// This mock is used to simulate an already active navigator
class MockNavigator : public nav2_core::NavigatorBase
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

  void preempt() override {preempt_called_ = true;}

  bool preempt_called_ = false;
};

TEST(BTNavigatorOnGoalReceivedTest, AlreadyNavigatingPreemptionDisabledGoalRejected)
{
  nav2_core::NavigatorMuxer muxer;
  MockNavigator active_nav;
  muxer.startNavigating(&active_nav);

  TestBTNavigator nav;
  nav.setup(&muxer, false);

  auto goal = std::make_shared<NavigateToPose::Goal>();
  EXPECT_FALSE(nav.callOnGoalReceived(goal));
  EXPECT_FALSE(active_nav.preempt_called_);
  // Verify active_nav still in progress
  EXPECT_TRUE(muxer.isNavigating());
  muxer.stopNavigating(&active_nav);
  EXPECT_FALSE(muxer.isNavigating());
}

TEST(BTNavigatorOnGoalReceivedTest, AlreadyNavigatingPreemptionEnabledGoalAccepted)
{
  nav2_core::NavigatorMuxer muxer;
  MockNavigator active_nav;
  muxer.startNavigating(&active_nav);

  // Simulate active_nav finishing after preempt() is triggered.
  std::thread stopper(
    [&]() {
      while (!active_nav.preempt_called_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      muxer.stopNavigating(&active_nav);
    });

  TestBTNavigator nav;
  nav.setup(&muxer, true, 500);

  auto goal = std::make_shared<NavigateToPose::Goal>();
  EXPECT_TRUE(nav.callOnGoalReceived(goal));
  stopper.join();
  EXPECT_TRUE(active_nav.preempt_called_);
  // Verify nav is in progress instead of active_nav
  EXPECT_TRUE(muxer.isNavigating());
  muxer.stopNavigating(&nav);
  EXPECT_FALSE(muxer.isNavigating());
}

TEST(BTNavigatorOnGoalReceivedTest, AlreadyNavigatingPreemptionEnabledTimeout)
{
  nav2_core::NavigatorMuxer muxer;
  MockNavigator active_nav;
  muxer.startNavigating(&active_nav);

  TestBTNavigator nav;
  // 50ms timeout so that we don't block for too long
  nav.setup(&muxer, true, 50);

  auto goal = std::make_shared<NavigateToPose::Goal>();
  EXPECT_FALSE(nav.callOnGoalReceived(goal));
  EXPECT_TRUE(active_nav.preempt_called_);

  // Verify active_nav is in progress instead of nav because goal rejected
  EXPECT_TRUE(muxer.isNavigating());
  muxer.stopNavigating(&active_nav);
  EXPECT_FALSE(muxer.isNavigating());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
