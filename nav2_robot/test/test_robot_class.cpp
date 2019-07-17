// Copyright (c) 2018 Intel Corporation
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
#include <string>
#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_robot/robot.hpp"
#include "nav2_util/lifecycle_service_client.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using lifecycle_msgs::msg::Transition;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};

RclCppFixture g_rclcppfixture;

class TestLifecycleNode : public nav2_util::LifecycleNode
{
public:
  TestLifecycleNode()
  : nav2_util::LifecycleNode("TestLifecycleNode")
  {
  }

  nav2_util::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override
  {
    robot_ = std::make_unique<nav2_robot::Robot>(shared_from_this());
    robot_->on_configure(state);
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override
  {
    robot_->on_activate(state);
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    robot_->on_deactivate(state);
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override
  {
    robot_->on_cleanup(state);
    robot_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  bool getOdometry(nav_msgs::msg::Odometry::SharedPtr & robot_odom)
  {
    return robot_->getOdometry(robot_odom);
  }

  std::string getName()
  {
    return robot_->getName();
  }

  bool getCurrentPose(
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & robot_pose)
  {
    return robot_->getCurrentPose(robot_pose);
  }

  void sendVelocity(geometry_msgs::msg::Twist twist)
  {
    robot_->sendVelocity(twist);
  }

protected:
  std::unique_ptr<nav2_robot::Robot> robot_;
};

class TestRobotClass : public ::testing::Test
{
public:
  TestRobotClass()
  {
    // Create a lifecycle node that uses a robot and start a thread to handle its messages
    lifecycle_node_ = std::make_shared<TestLifecycleNode>();

    lifecycle_thread_ = std::make_unique<std::thread>(
      [this](nav2_util::LifecycleNode::SharedPtr node) {
        for (;; ) {
          rclcpp::spin_some(node->get_node_base_interface());
          if (shut_down_threads_) {return;}
        }
      }, lifecycle_node_
    );

    client_ = std::make_shared<rclcpp::Node>("test_robot_client_node");

    lifecycle_client_ =
      std::make_shared<nav2_util::LifecycleServiceClient>("TestLifecycleNode", client_);

    lifecycle_client_->change_state(Transition::TRANSITION_CONFIGURE);
    lifecycle_client_->change_state(Transition::TRANSITION_ACTIVATE);

    // Initializing Pose and Twist messages
    initTestPose();
    initTestTwist();

    // Creating fake publishers
    pose_pub_ = client_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose", rclcpp::SystemDefaultsQoS().transient_local());

    odom_pub_ = client_->create_publisher<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SystemDefaultsQoS());

    // Subscribing to cmdVelocity topic to make sure Robot class is publishing velocity
    vel_sub_ = client_->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&TestRobotClass::velocityReceived, this, std::placeholders::_1));

    velocityCmdReceived_ = false;
  }

  ~TestRobotClass()
  {
    lifecycle_client_->change_state(Transition::TRANSITION_DEACTIVATE);
    lifecycle_client_->change_state(Transition::TRANSITION_CLEANUP);

    shut_down_threads_ = true;
    lifecycle_thread_->join();
  }

  void velocityReceived(const geometry_msgs::msg::Twist::SharedPtr msg);

protected:
  std::shared_ptr<rclcpp::Node> client_;

  std::shared_ptr<TestLifecycleNode> lifecycle_node_;
  std::unique_ptr<std::thread> lifecycle_thread_;

  std::shared_ptr<nav2_util::LifecycleServiceClient> lifecycle_client_;

  geometry_msgs::msg::PoseWithCovarianceStamped testPose_;
  geometry_msgs::msg::Twist testTwist_;
  nav_msgs::msg::Odometry testOdom_;
  geometry_msgs::msg::Twist velocityReceived_;

  bool velocityCmdReceived_;
  bool shut_down_threads_{false};

  void initTestPose();
  void initTestTwist();
  void publishPose();
  void publishOdom();

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::ConstSharedPtr vel_sub_;
};

void
TestRobotClass::publishPose()
{
  pose_pub_->publish(testPose_);
}

void
TestRobotClass::publishOdom()
{
  testOdom_.pose = testPose_.pose;
  testOdom_.twist.twist = testTwist_;
  testOdom_.header.frame_id = "base_footprint";
  testOdom_.header.stamp = rclcpp::Time();
  odom_pub_->publish(testOdom_);
}

void
TestRobotClass::velocityReceived(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  velocityReceived_ = *msg;
  velocityCmdReceived_ = true;
}

void TestRobotClass::initTestPose()
{
  testPose_.header.frame_id = "testPose";
  testPose_.header.stamp = rclcpp::Time();
  testPose_.pose.pose.position.x = 0.0;
  testPose_.pose.pose.position.y = 0.0;
  testPose_.pose.pose.position.z = 0.0;
  testPose_.pose.pose.orientation.x = 0.0;
  testPose_.pose.pose.orientation.y = 0.0;
  testPose_.pose.pose.orientation.z = 0.0;
  testPose_.pose.pose.orientation.w = 1.0;
  for (int i = 0; i < 12; i++) {
    testPose_.pose.covariance[i] = 0.0;
  }
}

void TestRobotClass::initTestTwist()
{
  testTwist_.linear.x = 0.0;
  testTwist_.linear.y = 0.0;
  testTwist_.linear.z = 0.0;
  testTwist_.angular.x = 0.0;
  testTwist_.angular.y = 0.0;
  testTwist_.angular.z = 0.0;
}

TEST_F(TestRobotClass, getNameTest)
{
  std::string robotName = lifecycle_node_->getName();
  EXPECT_EQ(robotName, "turtlebot");
}

TEST_F(TestRobotClass, getPoseTest)
{
  auto currentPose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  while (!(lifecycle_node_->getCurrentPose(currentPose))) {
    publishPose();
    rclcpp::spin_some(client_);
  }
  EXPECT_EQ(*currentPose, testPose_);
}

TEST_F(TestRobotClass, getOdometryTest)
{
  auto currentOdom = std::make_shared<nav_msgs::msg::Odometry>();
  rclcpp::Rate r(10);
  while (!(lifecycle_node_->getOdometry(currentOdom))) {
    publishOdom();
    r.sleep();
  }
  EXPECT_EQ(*currentOdom, testOdom_);
}

TEST_F(TestRobotClass, sendVelocityTest)
{
  rclcpp::Rate r(10);
  while (!velocityCmdReceived_) {
    lifecycle_node_->sendVelocity(testTwist_);
    rclcpp::spin_some(client_);
    r.sleep();
  }
  EXPECT_EQ(testTwist_, velocityReceived_);
}
