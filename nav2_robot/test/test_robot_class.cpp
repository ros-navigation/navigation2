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
#include "nav2_robot/ros_robot.hpp"
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
    node = rclcpp::Node::make_shared("robot_class_test");
    robot_ = std::make_unique<nav2_robot::RosRobot>(node.get());

    pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose");
    odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("odom");

    vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>("cmdVelocity",
        std::bind(&TestRobotClass::velocityReceived, this, std::placeholders::_1));

    velocityCmdReceived_ = false;
  }
  void velocityReceived(const geometry_msgs::msg::Twist::SharedPtr msg);

protected:
  std::shared_ptr<rclcpp::Node> node;
  std::unique_ptr<nav2_robot::RosRobot> robot_;
  void publishPose();
  void publishOdom();
  geometry_msgs::msg::PoseWithCovarianceStamped testPose_;
  nav_msgs::msg::Odometry testOdom_;
  geometry_msgs::msg::Twist velocityReceived_;
  bool velocityCmdReceived_;

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::ConstSharedPtr vel_sub_;
};

void
TestRobotClass::publishPose()
{
  testPose_.header.frame_id = "map";
  testPose_.header.stamp = rclcpp::Time();
  testPose_.pose.pose.position.x = 1.1;
  testPose_.pose.pose.position.y = 1.2;
  testPose_.pose.pose.position.z = 1.3;
  testPose_.pose.pose.orientation.x = 0.1;
  testPose_.pose.pose.orientation.y = 0.2;
  testPose_.pose.pose.orientation.z = 0.3;
  testPose_.pose.pose.orientation.w = 0.4;
  for (int i = 0; i < 12; i++) {
    testPose_.pose.covariance[i] = i;
  }
  pose_pub_->publish(testPose_);
}

void
TestRobotClass::publishOdom()
{
  testOdom_.header.frame_id = "base_footprint";
  testOdom_.header.stamp = rclcpp::Time();

  testOdom_.pose.pose.position.x = 1.1;
  testOdom_.pose.pose.position.y = 1.2;
  testOdom_.pose.pose.position.z = 1.3;
  testOdom_.pose.pose.orientation.x = 0.1;
  testOdom_.pose.pose.orientation.y = 0.2;
  testOdom_.pose.pose.orientation.z = 0.3;
  testOdom_.pose.pose.orientation.w = 0.4;
  for (int i = 0; i < 12; i++) {
    testOdom_.pose.covariance[i] = i;
  }

  testOdom_.twist.twist.linear.x = 1.1;
  testOdom_.twist.twist.linear.y = 1.2;
  testOdom_.twist.twist.linear.z = 1.3;
  testOdom_.twist.twist.angular.x = 1.1;
  testOdom_.twist.twist.angular.y = 1.2;
  testOdom_.twist.twist.angular.z = 1.3;

  for (int i = 0; i < 12; i++) {
    testOdom_.twist.covariance[i] = i;
  }

  odom_pub_->publish(testOdom_);
}

void
TestRobotClass::velocityReceived(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  velocityReceived_ = *msg;
  velocityCmdReceived_ = true;
}

TEST_F(TestRobotClass, getNameTest)
{
  std::string robotName = robot_->getRobotName();
  EXPECT_EQ(robotName, "turtlebot");
}

TEST_F(TestRobotClass, getPoseTest)
{
  auto currentPose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  while (!(robot_->getCurrentPose(currentPose))) {
    publishPose();
    rclcpp::spin_some(node);
  }
  EXPECT_EQ(*currentPose, testPose_);
}

TEST_F(TestRobotClass, getVelocityTest)
{
  auto currentOdom = std::make_shared<nav_msgs::msg::Odometry>();
  while (!(robot_->getCurrentVelocity(currentOdom))) {
    publishOdom();
    rclcpp::spin_some(node);
  }
  EXPECT_EQ(*currentOdom, testOdom_);
}

TEST_F(TestRobotClass, sendVelocityTest)
{
  geometry_msgs::msg::Twist velocity_cmd;
  velocity_cmd.linear.x = 1.1;
  velocity_cmd.linear.y = 1.2;
  velocity_cmd.linear.z = 1.3;
  velocity_cmd.angular.x = 1.1;
  velocity_cmd.angular.y = 1.2;
  velocity_cmd.angular.z = 1.3;

  robot_->sendVelocity(velocity_cmd);

  while (!velocityCmdReceived_) {
    rclcpp::spin_some(node);
  }
  EXPECT_EQ(velocity_cmd, velocityReceived_);
}
