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
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "nav2_amcl/amcl_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

// rclcpp::init can only be called once per process, so this needs to be a global variable
class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestAmclPose : public ::testing::Test
{
public:
  TestAmclPose()
  {
    pose_callback_ = false;
    initTestPose();
    tol_ = 0.25;

    node = rclcpp::Node::make_shared("localization_test");

    while (node->count_subscribers("scan") < 1) {
      std::this_thread::sleep_for(100ms);
      rclcpp::spin_some(node);
    }

    initial_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", rclcpp::SystemDefaultsQoS());
    subscription_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&TestAmclPose::amcl_pose_callback, this, _1));
    initial_pose_pub_->publish(testPose_);
  }

  bool defaultAmclTest();

protected:
  std::shared_ptr<rclcpp::Node> node;
  void initTestPose();

private:
  void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    auto amcl_pose = msg->pose;
    amcl_pose_x = amcl_pose.pose.position.x;
    amcl_pose_y = amcl_pose.pose.position.y;
    pose_callback_ = true;
  }
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
  geometry_msgs::msg::PoseWithCovarianceStamped testPose_;
  double amcl_pose_x;
  double amcl_pose_y;
  bool pose_callback_;
  float tol_;
};

bool TestAmclPose::defaultAmclTest()
{
  initial_pose_pub_->publish(testPose_);
  while (!pose_callback_) {
    // TODO(mhpanah): Initial pose should only be published once.
    initial_pose_pub_->publish(testPose_);
    std::this_thread::sleep_for(1s);
    rclcpp::spin_some(node);
  }
  if (std::abs(amcl_pose_x - testPose_.pose.pose.position.x) < tol_ &&
    std::abs(amcl_pose_y - testPose_.pose.pose.position.y) < tol_)
  {
    return true;
  } else {
    return false;
  }
}

void TestAmclPose::initTestPose()
{
  testPose_.header.frame_id = "map";
  testPose_.header.stamp = rclcpp::Time();
  testPose_.pose.pose.position.x = -2.0;
  testPose_.pose.pose.position.y = -0.5;
  testPose_.pose.pose.position.z = 0.0;
  testPose_.pose.pose.orientation.x = 0.0;
  testPose_.pose.pose.orientation.y = 0.0;
  testPose_.pose.pose.orientation.z = 0.0;
  testPose_.pose.pose.orientation.w = 1.0;
  for (int i = 0; i < 35; i++) {
    testPose_.pose.covariance[i] = 0.0;
  }
  testPose_.pose.covariance[0] = 0.08;
  testPose_.pose.covariance[7] = 0.08;
  testPose_.pose.covariance[35] = 0.05;
}

TEST_F(TestAmclPose, SimpleAmclTest)
{
  EXPECT_EQ(true, defaultAmclTest());
}
