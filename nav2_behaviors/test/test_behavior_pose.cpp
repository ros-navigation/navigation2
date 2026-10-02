// Copyright (c) 2026 Open Navigation LLC
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

#include <memory>
#include <string>
#include <type_traits>

#include "gtest/gtest.h"
#include "nav2_behaviors/plugins/assisted_teleop.hpp"
#include "nav2_behaviors/plugins/back_up.hpp"
#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav2_behaviors/plugins/spin.hpp"

template<typename Plugin, typename Action>
class TestBehavior : public Plugin
{
public:
  using ActionType = Action;
  using Plugin::getCurrentPoseChecked;
};

using BehaviorTypes = ::testing::Types<
  TestBehavior<nav2_behaviors::Spin, nav2_msgs::action::Spin>,
  TestBehavior<nav2_behaviors::DriveOnHeading<>, nav2_msgs::action::DriveOnHeading>,
  TestBehavior<nav2_behaviors::BackUp, nav2_msgs::action::BackUp>,
  TestBehavior<nav2_behaviors::AssistedTeleop, nav2_msgs::action::AssistedTeleop>>;

template<typename Behavior>
class BehaviorPoseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<nav2::LifecycleNode>("behavior_pose_test");
    node_->declare_parameter("local_frame", "odom");
    node_->declare_parameter("global_frame", "map");
    node_->declare_parameter("robot_base_frame", "base_link");
    node_->declare_parameter("cycle_frequency", 10.0);
    node_->declare_parameter("transform_staleness_threshold", 10.0);
    // These tests exercise pose acquisition without requiring a costmap.
    node_->declare_parameter("projection_time", 0.0);
    tf_ = std::make_shared<nav2::TransformBuffer>(node_->get_clock());
    behavior_ = std::make_unique<Behavior>();
    behavior_->configure(node_, "behavior", tf_, nullptr, nullptr);
    behavior_->activate();
  }

  void TearDown() override
  {
    behavior_->deactivate();
    behavior_->cleanup();
  }

  geometry_msgs::msg::TransformStamped setTransform(double age, bool is_static = false)
  {
    tf_->clear();
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.header.stamp = node_->now() - rclcpp::Duration::from_seconds(age);
    transform.transform.translation.x = 1.0;
    transform.transform.translation.y = 2.0;
    transform.transform.rotation.w = 1.0;
    EXPECT_TRUE(tf_->setTransform(transform, "test", is_static));
    return transform;
  }

  nav2::LifecycleNode::SharedPtr node_;
  nav2::TransformBuffer::SharedPtr tf_;
  std::unique_ptr<Behavior> behavior_;
};

TYPED_TEST_SUITE(BehaviorPoseTest, BehaviorTypes);

TYPED_TEST(BehaviorPoseTest, CheckedPosePreservesTimestampAndRejectsStaleness)
{
  geometry_msgs::msg::PoseStamped pose;
  EXPECT_FALSE(this->behavior_->getCurrentPoseChecked(pose));

  const auto transform = this->setTransform(1.0);
  ASSERT_TRUE(this->behavior_->getCurrentPoseChecked(pose));
  EXPECT_EQ(pose.header, transform.header);
  EXPECT_DOUBLE_EQ(pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.pose.position.y, 2.0);
  EXPECT_EQ(pose.pose.orientation, transform.transform.rotation);

  const auto previous_pose = pose;
  this->setTransform(30.0);
  EXPECT_FALSE(this->behavior_->getCurrentPoseChecked(pose));
  EXPECT_EQ(pose, previous_pose);

  this->setTransform(30.0, true);
  EXPECT_TRUE(this->behavior_->getCurrentPoseChecked(pose));
}

void prepareGoal(nav2_msgs::action::Spin::Goal & goal)
{
  goal.target_yaw = 1.0;
  goal.disable_collision_checks = true;
}

void prepareGoal(nav2_msgs::action::DriveOnHeading::Goal & goal)
{
  goal.target.x = 1.0;
  goal.speed = 0.2;
  goal.disable_collision_checks = true;
}

void prepareGoal(nav2_msgs::action::BackUp::Goal & goal)
{
  goal.target.x = 1.0;
  goal.speed = 0.2;
  goal.disable_collision_checks = true;
}

void prepareGoal(nav2_msgs::action::AssistedTeleop::Goal &) {}

TYPED_TEST(BehaviorPoseTest, RechecksPoseEachCycle)
{
  using Action = typename TypeParam::ActionType;
  auto goal = std::make_shared<typename Action::Goal>();
  prepareGoal(*goal);
  this->setTransform(1.0);
  ASSERT_EQ(this->behavior_->onRun(goal).status, nav2_behaviors::Status::SUCCEEDED);
  EXPECT_EQ(this->behavior_->onCycleUpdate().status, nav2_behaviors::Status::RUNNING);

  this->setTransform(30.0);
  const auto stale_result = this->behavior_->onCycleUpdate();
  EXPECT_EQ(stale_result.status, nav2_behaviors::Status::FAILED);
  EXPECT_EQ(stale_result.error_code, Action::Result::TF_ERROR);

  this->tf_->clear();
  const auto missing_result = this->behavior_->onCycleUpdate();
  EXPECT_EQ(missing_result.status, nav2_behaviors::Status::FAILED);
  EXPECT_EQ(missing_result.error_code, Action::Result::TF_ERROR);
}

TYPED_TEST(BehaviorPoseTest, RejectsStaleInitialPose)
{
  using Action = typename TypeParam::ActionType;
  if constexpr (!std::is_same_v<Action, nav2_msgs::action::AssistedTeleop>) {
    auto goal = std::make_shared<typename Action::Goal>();
    prepareGoal(*goal);
    this->setTransform(30.0);
    const auto result = this->behavior_->onRun(goal);
    EXPECT_EQ(result.status, nav2_behaviors::Status::FAILED);
    EXPECT_EQ(result.error_code, Action::Result::TF_ERROR);
  }
}

TYPED_TEST(BehaviorPoseTest, ZeroThresholdDisablesAgeCheck)
{
  this->behavior_->deactivate();
  this->behavior_->cleanup();
  this->node_->set_parameter(rclcpp::Parameter("transform_staleness_threshold", 0.0));
  this->behavior_->configure(this->node_, "behavior", this->tf_, nullptr, nullptr);
  this->behavior_->activate();

  this->setTransform(30.0);
  geometry_msgs::msg::PoseStamped pose;
  EXPECT_TRUE(this->behavior_->getCurrentPoseChecked(pose));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
