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

#include <memory>
#include <type_traits>

#include "nav2_bt_navigator/navigators/navigate_to_pose.hpp"
#include "nav2_bt_navigator/navigators/navigate_through_poses.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// Exercise the real navigator callbacks and blackboard without spinning a BT.
template<class NavigatorT>
class TestNavigator : public NavigatorT
{
public:
  using ActionT = typename NavigatorT::ActionT;
  using NavigatorT::goalReceived;
  using NavigatorT::goalStarted;
  using NavigatorT::goalCompleted;
  using NavigatorT::onPreempt;

  BT::Blackboard::Ptr blackboard() {return this->bt_action_server_->getBlackboard();}
  rclcpp::Time startTime() {return this->start_time_;}
  bool hasInternalError()
  {
    auto result = std::make_shared<typename ActionT::Result>();
    return this->bt_action_server_->populateInternalError(result);
  }
};

template<class NavigatorT>
class GoalReceptionTest : public ::testing::Test
{
public:
  using ActionT = typename NavigatorT::ActionT;

  void SetUp() override
  {
    node = std::make_shared<nav2::LifecycleNode>("goal_reception_test");
    tf = std::make_shared<nav2::TransformBuffer>(node->get_clock());
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf->setTransform(transform, "test", true));
    nav2_core::FeedbackUtils feedback{"base_link", "map", 0.0, tf};
    ASSERT_TRUE(nav.on_configure(node, {}, feedback, &muxer, nullptr));
  }

  void TearDown() override
  {
    nav.on_cleanup();
    tf.reset();
    node.reset();
  }

  std::shared_ptr<typename ActionT::Goal> goal(double x, bool valid = true)
  {
    auto result = std::make_shared<typename ActionT::Goal>();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = valid ? "map" : "missing_frame";
    pose.pose.position.x = x;
    pose.pose.orientation.w = 1.0;
    if constexpr (std::is_same_v<ActionT, nav2_msgs::action::NavigateToPose>) {
      result->pose = pose;
    } else {
      result->poses.goals.push_back(pose);
    }
    return result;
  }

  double activeX()
  {
    if constexpr (std::is_same_v<ActionT, nav2_msgs::action::NavigateToPose>) {
      return nav.blackboard()->template get<geometry_msgs::msg::PoseStamped>(
        "goal").pose.position.x;
    } else {
      return nav.blackboard()->template get<nav_msgs::msg::Goals>(
        "goals").goals.front().pose.position.x;
    }
  }

  nav2::LifecycleNode::SharedPtr node;
  nav2::TransformBuffer::SharedPtr tf;
  nav2_core::NavigatorMuxer muxer;
  TestNavigator<NavigatorT> nav;
};

using Navigators = ::testing::Types<
  nav2_bt_navigator::NavigateToPoseNavigator,
  nav2_bt_navigator::NavigateThroughPosesNavigator>;
TYPED_TEST_SUITE(GoalReceptionTest, Navigators);

TYPED_TEST(GoalReceptionTest, ReceptionPreservesRunningGoalAndFeedback)
{
  ASSERT_TRUE(this->nav.goalStarted(this->goal(1.0)));
  this->nav.blackboard()->set("number_recoveries", 7);
  const auto start = this->nav.startTime();
  auto replacement = this->goal(9.0);
  replacement->behavior_tree = "different_tree.xml";
  EXPECT_TRUE(this->nav.goalReceived(replacement));
  EXPECT_DOUBLE_EQ(this->activeX(), 1.0);
  EXPECT_EQ(this->nav.startTime(), start);
  EXPECT_EQ(this->nav.blackboard()->template get<int>("number_recoveries"), 7);
  EXPECT_FALSE(this->nav.hasInternalError());
  this->nav.onPreempt(replacement);
  EXPECT_DOUBLE_EQ(this->activeX(), 1.0);
  EXPECT_EQ(this->nav.startTime(), start);
  EXPECT_EQ(this->nav.blackboard()->template get<int>("number_recoveries"), 7);
}

TYPED_TEST(GoalReceptionTest, InvalidTFRejectedWithoutContaminatingActiveResult)
{
  ASSERT_TRUE(this->nav.goalStarted(this->goal(1.0)));
  this->nav.blackboard()->set("number_recoveries", 7);
  const auto start = this->nav.startTime();
  EXPECT_FALSE(this->nav.goalReceived(this->goal(9.0, false)));
  EXPECT_DOUBLE_EQ(this->activeX(), 1.0);
  EXPECT_EQ(this->nav.startTime(), start);
  EXPECT_EQ(this->nav.blackboard()->template get<int>("number_recoveries"), 7);
  EXPECT_FALSE(this->nav.hasInternalError());
}

TYPED_TEST(GoalReceptionTest, TFBecomingInvalidBeforePreemptionPreservesActiveState)
{
  ASSERT_TRUE(this->nav.goalStarted(this->goal(1.0)));
  this->nav.blackboard()->set("number_recoveries", 7);
  const auto start = this->nav.startTime();
  // A pending request already validated at reception can lose its robot TF later.
  auto replacement = this->goal(9.0);
  ASSERT_TRUE(this->nav.goalReceived(replacement));
  this->tf->clear();
  EXPECT_NO_THROW(this->nav.onPreempt(replacement));
  EXPECT_DOUBLE_EQ(this->activeX(), 1.0);
  EXPECT_EQ(this->nav.startTime(), start);
  EXPECT_EQ(this->nav.blackboard()->template get<int>("number_recoveries"), 7);
  EXPECT_FALSE(this->nav.hasInternalError());
}

using ThroughPosesCompletionTest =
  GoalReceptionTest<nav2_bt_navigator::NavigateThroughPosesNavigator>;

TEST_F(ThroughPosesCompletionTest, BTLoadFailureBeforeInitializationPreservesError)
{
  auto result = std::make_shared<ActionT::Result>();
  result->error_code = ActionT::Result::FAILED_TO_LOAD_BEHAVIOR_TREE;
  result->error_msg = "Missing behavior tree";
  auto status = nav2_behavior_tree::BtStatus::FAILED;
  EXPECT_NO_THROW(nav.goalCompleted(result, status));
  EXPECT_EQ(result->error_code, ActionT::Result::FAILED_TO_LOAD_BEHAVIOR_TREE);
  EXPECT_EQ(result->error_msg, "Missing behavior tree");
  EXPECT_TRUE(result->waypoint_statuses.empty());
}

TEST_F(ThroughPosesCompletionTest, FailedInitializationPreservesTFError)
{
  EXPECT_FALSE(nav.goalStarted(goal(9.0, false)));
  auto result = std::make_shared<ActionT::Result>();
  auto status = nav2_behavior_tree::BtStatus::FAILED;
  EXPECT_NO_THROW(nav.goalCompleted(result, status));
  EXPECT_EQ(result->error_code, ActionT::Result::TF_ERROR);
  EXPECT_TRUE(result->waypoint_statuses.empty());
}

TEST_F(ThroughPosesCompletionTest, BTLoadFailureDoesNotReuseCompletedGoalWaypoints)
{
  ASSERT_TRUE(nav.goalStarted(goal(1.0)));
  auto completed = std::make_shared<ActionT::Result>();
  auto status = nav2_behavior_tree::BtStatus::SUCCEEDED;
  nav.goalCompleted(completed, status);
  ASSERT_EQ(completed->waypoint_statuses.size(), 1u);
  EXPECT_EQ(
    completed->waypoint_statuses.front().waypoint_status,
    nav2_msgs::msg::WaypointStatus::COMPLETED);

  auto failed = std::make_shared<ActionT::Result>();
  failed->error_code = ActionT::Result::FAILED_TO_LOAD_BEHAVIOR_TREE;
  status = nav2_behavior_tree::BtStatus::FAILED;
  EXPECT_NO_THROW(nav.goalCompleted(failed, status));
  EXPECT_EQ(failed->error_code, ActionT::Result::FAILED_TO_LOAD_BEHAVIOR_TREE);
  EXPECT_TRUE(failed->waypoint_statuses.empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
