// Copyright (c) 2019 Intel Corporation
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

#include <string>
#include <memory>
#include <chrono>
#include <iostream>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/dummy_behavior.hpp"

using nav2_behaviors::TimedBehavior;
using nav2_behaviors::Status;
using nav2_behaviors::ResultStatus;
using BehaviorAction = nav2_msgs::action::DummyBehavior;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<BehaviorAction>;

using namespace std::chrono_literals;

// A behavior for testing the base class

class DummyBehavior : public TimedBehavior<BehaviorAction>
{
  using CostmapInfoType = nav2_core::CostmapInfoType;

public:
  DummyBehavior()
  : TimedBehavior<BehaviorAction>(),
    initialized_(false) {}

  ~DummyBehavior() = default;

  ResultStatus onRun(const std::shared_ptr<const BehaviorAction::Goal> goal) override
  {
    // A normal behavior would catch the command and initialize
    initialized_ = false;
    command_ = goal->command.data;
    start_time_ = std::chrono::system_clock::now();

    // onRun method can have various possible outcomes (success, failure, cancelled)
    // The output is defined by the tester class on the command string.
    if (command_ == "Testing success" || command_ == "Testing failure on run") {
      initialized_ = true;
      return ResultStatus{Status::SUCCEEDED, 0};
    }

    return ResultStatus{Status::FAILED, 0};
  }

  ResultStatus onCycleUpdate() override
  {
    // A normal behavior would set the robot in motion in the first call
    // and check for robot states on subsequent calls to check if the movement
    // was completed.

    if (command_ != "Testing success" || !initialized_) {
      return ResultStatus{Status::FAILED, 0};
    }

    // For testing, pretend the robot takes some fixed
    // amount of time to complete the motion.
    auto current_time = std::chrono::system_clock::now();
    auto motion_duration = 1s;

    if (current_time - start_time_ >= motion_duration) {
      // Movement was completed
      return ResultStatus{Status::SUCCEEDED, 0};
    }

    return ResultStatus{Status::RUNNING, 0};
  }

  /**
   * @brief Method to determine the required costmap info
   * @return costmap resources needed
   */
  CostmapInfoType getResourceInfo() override {return CostmapInfoType::LOCAL;}

private:
  bool initialized_;
  std::string command_;
  std::chrono::system_clock::time_point start_time_;
};

// Define a test class to hold the context for the tests

class BehaviorTest : public ::testing::Test
{
protected:
  BehaviorTest() = default;
  ~BehaviorTest() = default;

  void SetUp() override
  {
    node_lifecycle_ =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "LifecycleBehaviorTestNode", rclcpp::NodeOptions());
    node_lifecycle_->declare_parameter(
      "local_costmap_topic",
      rclcpp::ParameterValue(std::string("local_costmap/costmap_raw")));
    node_lifecycle_->declare_parameter(
      "local_footprint_topic",
      rclcpp::ParameterValue(std::string("local_costmap/published_footprint")));

    node_lifecycle_->declare_parameter(
      "global_costmap_topic",
      rclcpp::ParameterValue(std::string("global_costmap/costmap_raw")));
    node_lifecycle_->declare_parameter(
      "global_footprint_topic",
      rclcpp::ParameterValue(std::string("global_costmap/published_footprint")));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_lifecycle_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node_lifecycle_->get_node_base_interface(),
      node_lifecycle_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::string local_costmap_topic, global_costmap_topic;
    std::string local_footprint_topic, global_footprint_topic;
    node_lifecycle_->get_parameter("local_costmap_topic", local_costmap_topic);
    node_lifecycle_->get_parameter("global_costmap_topic", global_costmap_topic);
    node_lifecycle_->get_parameter("local_footprint_topic", local_footprint_topic);
    node_lifecycle_->get_parameter("global_footprint_topic", global_footprint_topic);

    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> local_costmap_sub_ =
      std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
      node_lifecycle_, global_costmap_topic);

    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> global_costmap_sub_ =
      std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
      node_lifecycle_, global_costmap_topic);

    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> local_footprint_sub_ =
      std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
      node_lifecycle_, local_footprint_topic, *tf_buffer_);

    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> global_footprint_sub_ =
      std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
      node_lifecycle_, global_footprint_topic, *tf_buffer_);

    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> local_collision_checker_ =
      std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
      *local_costmap_sub_, *local_footprint_sub_,
      node_lifecycle_->get_name());

    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> global_collision_checker_ =
      std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
      *global_costmap_sub_, *global_footprint_sub_,
      node_lifecycle_->get_name());

    behavior_ = std::make_shared<DummyBehavior>();
    behavior_->configure(
      node_lifecycle_,
      "Behavior",
      tf_buffer_,
      local_collision_checker_,
      global_collision_checker_);
    behavior_->activate();

    client_ = rclcpp_action::create_client<BehaviorAction>(
      node_lifecycle_->get_node_base_interface(),
      node_lifecycle_->get_node_graph_interface(),
      node_lifecycle_->get_node_logging_interface(),
      node_lifecycle_->get_node_waitables_interface(), "Behavior");
    std::cout << "Setup complete." << std::endl;
  }

  void TearDown() override {}

  bool sendCommand(const std::string & command)
  {
    if (!client_->wait_for_action_server(4s)) {
      std::cout << "Server not up" << std::endl;
      return false;
    }

    auto goal = BehaviorAction::Goal();
    goal.command.data = command;
    auto future_goal = client_->async_send_goal(goal);

    if (rclcpp::spin_until_future_complete(node_lifecycle_, future_goal) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      std::cout << "failed sending goal" << std::endl;
      // failed sending the goal
      return false;
    }

    goal_handle_ = future_goal.get();

    if (!goal_handle_) {
      std::cout << "goal was rejected" << std::endl;
      // goal was rejected by the action server
      return false;
    }

    return true;
  }

  Status getOutcome()
  {
    if (getResult().code == rclcpp_action::ResultCode::SUCCEEDED) {
      return Status::SUCCEEDED;
    }

    return Status::FAILED;
  }

  ClientGoalHandle::WrappedResult getResult()
  {
    std::cout << "Getting async result..." << std::endl;
    auto future_result = client_->async_get_result(goal_handle_);
    std::cout << "Waiting on future..." << std::endl;
    rclcpp::spin_until_future_complete(node_lifecycle_, future_result);
    std::cout << "future received!" << std::endl;
    return future_result.get();
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_lifecycle_;
  std::shared_ptr<DummyBehavior> behavior_;
  std::shared_ptr<rclcpp_action::Client<BehaviorAction>> client_;
  std::shared_ptr<rclcpp_action::ClientGoalHandle<BehaviorAction>> goal_handle_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

// Define the tests

TEST_F(BehaviorTest, testingSuccess)
{
  ASSERT_TRUE(sendCommand("Testing success"));
  EXPECT_EQ(getOutcome(), Status::SUCCEEDED);
  SUCCEED();
}

TEST_F(BehaviorTest, testingFailureOnRun)
{
  ASSERT_TRUE(sendCommand("Testing failure on run"));
  EXPECT_EQ(getOutcome(), Status::FAILED);
  SUCCEED();
}

TEST_F(BehaviorTest, testingFailureOnInit)
{
  ASSERT_TRUE(sendCommand("Testing failure on init"));
  EXPECT_EQ(getOutcome(), Status::FAILED);
  SUCCEED();
}

TEST_F(BehaviorTest, testingSequentialFailures)
{
  ASSERT_TRUE(sendCommand("Testing failure on run"));
  EXPECT_EQ(getOutcome(), Status::FAILED);
  SUCCEED();
}

TEST_F(BehaviorTest, testingTotalElapsedTimeIsGratherThanZeroIfStarted)
{
  ASSERT_TRUE(sendCommand("Testing success"));
  EXPECT_GT(getResult().result->total_elapsed_time.sec, 0.0);
  SUCCEED();
}

TEST_F(BehaviorTest, testingTotalElapsedTimeIsZeroIfFailureOnInit)
{
  ASSERT_TRUE(sendCommand("Testing failure on init"));
  EXPECT_EQ(getResult().result->total_elapsed_time.sec, 0.0);
  SUCCEED();
}

TEST_F(BehaviorTest, testingTotalElapsedTimeIsZeroIfFailureOnRun)
{
  ASSERT_TRUE(sendCommand("Testing failure on run"));
  EXPECT_EQ(getResult().result->total_elapsed_time.sec, 0.0);
  SUCCEED();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
