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
#include "rclcpp/rclcpp.hpp"
#include "nav2_robot/robot.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestRobotClass : public ::testing::Test
{
public:
  TestRobotClass()
  {
    node_ = rclcpp::Node::make_shared("robot_class_test");
    robot_ = std::make_unique<nav2_robot::Robot>(node_.get());

    // Initializing Pose and Twist messages
    initTestPose();
    initTestTwist();

    // Creating fake publishers
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose");
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom");

    // Subscribing to cmdVelocity topic to make sure Robot class is publishing velocity
    vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("cmdVelocity",
        std::bind(&TestRobotClass::velocityReceived, this, std::placeholders::_1));

    velocityCmdReceived_ = false;
  }
  void velocityReceived(const geometry_msgs::msg::Twist::SharedPtr msg);

protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<nav2_robot::Robot> robot_;

  geometry_msgs::msg::PoseWithCovarianceStamped testPose_;
  geometry_msgs::msg::Twist testTwist_;
  nav_msgs::msg::Odometry testOdom_;
  geometry_msgs::msg::Twist velocityReceived_;

  bool velocityCmdReceived_;

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
  std::string robotName = robot_->getName();
  EXPECT_EQ(robotName, "turtlebot");
}

TEST_F(TestRobotClass, getPoseTest)
{
  auto currentPose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  while (!(robot_->getCurrentPose(currentPose))) {
    publishPose();
    rclcpp::spin_some(node_);
  }
  EXPECT_EQ(*currentPose, testPose_);
}

TEST_F(TestRobotClass, getVelocityTest)
{
  auto currentOdom = std::make_shared<nav_msgs::msg::Odometry>();
  while (!(robot_->getCurrentVelocity(currentOdom))) {
    publishOdom();
    rclcpp::spin_some(node_);
  }
  EXPECT_EQ(*currentOdom, testOdom_);
}

TEST_F(TestRobotClass, sendVelocityTest)
{
  while (!velocityCmdReceived_) {
    robot_->sendVelocity(testTwist_);
    rclcpp::spin_some(node_);
  }
  EXPECT_EQ(testTwist_, velocityReceived_);
}
