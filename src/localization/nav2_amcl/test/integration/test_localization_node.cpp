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
#include "amcl_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

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
    // Initializing amcl_pose x and y to NaN
    amcl_pose_x = 0.0 / 0.0;
    amcl_pose_y = 0.0 / 0.0;
    pose_callback_ = false;

    node = rclcpp::Node::make_shared("localization_test");

    while (node->count_subscribers("/scan") < 1) {
      rclcpp::spin_some(node);
    }

    subscription_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose",
      std::bind(&TestAmclPose::amcl_pose_callback, this, _1));
  }
  bool defaultAmclTest();

protected:
  std::shared_ptr<rclcpp::Node> node;

private:
  void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    auto amcl_pose = msg->pose;
    amcl_pose_x = amcl_pose.pose.position.x;
    amcl_pose_y = amcl_pose.pose.position.y;
    pose_callback_ = true;
  }
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
  double amcl_pose_x;
  double amcl_pose_y;
  bool pose_callback_;
};

bool TestAmclPose::defaultAmclTest()
{
  while (!pose_callback_) {
    rclcpp::spin_some(node);
  }
  if (!std::isnan(amcl_pose_x) && !std::isnan(amcl_pose_y)) {
    return true;
  } else {
    return false;
  }
}

TEST_F(TestAmclPose, SimpleAmclTest)
{
  EXPECT_EQ(true, defaultAmclTest());
}
