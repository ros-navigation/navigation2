// Copyright (c) 2026, Dexory (Tony Najjar)
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
#include <chrono>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_loopback_sim/loopback_simulator.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"

using namespace std::chrono_literals;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class LoopbackSimulatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    // Disable sim_time and scan for simpler testing; use TwistStamped (default)
    options.parameter_overrides(
    {
      rclcpp::Parameter("use_sim_time", false),
      rclcpp::Parameter("publish_scan", false),
      rclcpp::Parameter("publish_clock", false),
      rclcpp::Parameter("update_duration", 0.01),
      rclcpp::Parameter("base_frame_id", "base_footprint"),
      rclcpp::Parameter("odom_frame_id", "odom"),
      rclcpp::Parameter("map_frame_id", "map"),
    });

    sim_node_ = std::make_shared<nav2_loopback_sim::LoopbackSimulator>(options);
    helper_node_ = rclcpp::Node::make_shared("test_helper");

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(sim_node_->get_node_base_interface());
    executor_->add_node(helper_node_);

    // TwistSubscriber defaults to TwistStamped (enable_stamped_cmd_vel=true)
    cmd_vel_pub_ = helper_node_->create_publisher<
      geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    initial_pose_pub_ = helper_node_->create_publisher<
      geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
  }

  void TearDown() override
  {
    executor_->cancel();
    cmd_vel_pub_.reset();
    initial_pose_pub_.reset();
    helper_node_.reset();
    sim_node_.reset();
    executor_.reset();
  }

  void configureAndActivate()
  {
    sim_node_->configure();
    sim_node_->activate();
    spinFor(200ms);
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      executor_->spin_some(10ms);
    }
  }

  void publishInitialPose(double x, double y, double yaw)
  {
    auto msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
    msg->header.stamp = helper_node_->now();
    msg->header.frame_id = "map";
    msg->pose.pose.position.x = x;
    msg->pose.pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    msg->pose.pose.orientation.x = q.x();
    msg->pose.pose.orientation.y = q.y();
    msg->pose.pose.orientation.z = q.z();
    msg->pose.pose.orientation.w = q.w();
    initial_pose_pub_->publish(std::move(msg));
  }

  void publishCmdVel(double vx, double vy, double wz)
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = helper_node_->now();
    msg->twist.linear.x = vx;
    msg->twist.linear.y = vy;
    msg->twist.angular.z = wz;
    cmd_vel_pub_->publish(std::move(msg));
  }

  std::shared_ptr<nav2_loopback_sim::LoopbackSimulator> sim_node_;
  rclcpp::Node::SharedPtr helper_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
};

// Verify that the node transitions through all lifecycle states correctly
TEST_F(LoopbackSimulatorTest, LifecycleTransitions)
{
  auto result = sim_node_->configure();
  EXPECT_EQ(result.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  result = sim_node_->activate();
  EXPECT_EQ(result.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  result = sim_node_->deactivate();
  EXPECT_EQ(result.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  result = sim_node_->cleanup();
  EXPECT_EQ(result.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  result = sim_node_->shutdown();
  EXPECT_EQ(result.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
}

// Verify that activating the node starts publishing odom->base_footprint transforms
TEST_F(LoopbackSimulatorTest, PublishesTfOnActivation)
{
  configureAndActivate();

  bool received_tf = false;
  auto tf_sub = helper_node_->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 10,
    [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
      for (const auto & t : msg->transforms) {
        if (t.header.frame_id == "odom" && t.child_frame_id == "base_footprint") {
          received_tf = true;
        }
      }
    });

  spinFor(500ms);
  EXPECT_TRUE(received_tf);
}

// Verify that publishing an initial pose creates a map->odom transform at the given position
TEST_F(LoopbackSimulatorTest, InitialPoseSetsMapToOdom)
{
  configureAndActivate();

  std::vector<geometry_msgs::msg::TransformStamped> received_tfs;
  auto tf_sub = helper_node_->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 10,
    [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
      for (const auto & t : msg->transforms) {
        received_tfs.push_back(t);
      }
    });

  publishInitialPose(1.0, 2.0, 0.0);
  spinFor(500ms);

  bool found_map_odom = false;
  for (const auto & t : received_tfs) {
    if (t.header.frame_id == "map" && t.child_frame_id == "odom") {
      found_map_odom = true;
      EXPECT_NEAR(t.transform.translation.x, 1.0, 0.1);
      EXPECT_NEAR(t.transform.translation.y, 2.0, 0.1);
    }
  }
  EXPECT_TRUE(found_map_odom);
}

// Verify that a forward cmd_vel causes the robot's odometry position to advance
TEST_F(LoopbackSimulatorTest, CmdVelMovesRobot)
{
  configureAndActivate();

  // Subscribe to odom before publishing
  std::vector<nav_msgs::msg::Odometry> odom_msgs;
  auto odom_sub = helper_node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
      odom_msgs.push_back(*msg);
    });

  publishInitialPose(0.0, 0.0, 0.0);
  spinFor(500ms);

  // Drive forward at 1.0 m/s for ~1.5s
  for (int i = 0; i < 50; i++) {
    publishCmdVel(1.0, 0.0, 0.0);
    spinFor(30ms);
  }

  ASSERT_FALSE(odom_msgs.empty());
  EXPECT_GT(odom_msgs.back().pose.pose.position.x, 0.0);
}

// Verify that the published odometry twist matches the commanded velocity
TEST_F(LoopbackSimulatorTest, OdometryContainsTwist)
{
  configureAndActivate();

  nav_msgs::msg::Odometry latest_odom;
  bool got_odom = false;
  auto odom_sub = helper_node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
      latest_odom = *msg;
      got_odom = true;
    });

  publishInitialPose(0.0, 0.0, 0.0);
  spinFor(500ms);

  for (int i = 0; i < 30; i++) {
    publishCmdVel(0.5, 0.0, 0.1);
    spinFor(30ms);
  }

  ASSERT_TRUE(got_odom);
  EXPECT_NEAR(latest_odom.twist.twist.linear.x, 0.5, 0.01);
  EXPECT_NEAR(latest_odom.twist.twist.angular.z, 0.1, 0.01);
}

// Verify that angular cmd_vel rotates the robot (changes yaw in the odom->base TF)
TEST_F(LoopbackSimulatorTest, RotationUpdatesYaw)
{
  configureAndActivate();

  geometry_msgs::msg::TransformStamped latest_base_tf;
  bool got_base_tf = false;
  auto tf_sub = helper_node_->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 10,
    [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
      for (const auto & t : msg->transforms) {
        if (t.child_frame_id == "base_footprint") {
          latest_base_tf = t;
          got_base_tf = true;
        }
      }
    });

  publishInitialPose(0.0, 0.0, 0.0);
  spinFor(500ms);

  // Rotate at 1 rad/s
  for (int i = 0; i < 50; i++) {
    publishCmdVel(0.0, 0.0, 1.0);
    spinFor(30ms);
  }

  ASSERT_TRUE(got_base_tf);

  tf2::Quaternion q(
    latest_base_tf.transform.rotation.x,
    latest_base_tf.transform.rotation.y,
    latest_base_tf.transform.rotation.z,
    latest_base_tf.transform.rotation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  EXPECT_GT(std::abs(yaw), 0.1);
}

// Verify that deactivating the node stops odometry publication
TEST_F(LoopbackSimulatorTest, DeactivateStopsPublishing)
{
  configureAndActivate();

  publishInitialPose(0.0, 0.0, 0.0);
  spinFor(500ms);

  sim_node_->deactivate();
  spinFor(100ms);

  int msg_count = 0;
  auto odom_sub = helper_node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    [&](const nav_msgs::msg::Odometry::SharedPtr) {
      msg_count++;
    });

  publishCmdVel(1.0, 0.0, 0.0);
  spinFor(300ms);

  EXPECT_EQ(msg_count, 0);
}

// Verify that speed_factor can be changed dynamically and non-positive values are rejected
TEST_F(LoopbackSimulatorTest, SpeedFactorDynamicReconfigure)
{
  configureAndActivate();

  // Valid change
  auto result = sim_node_->set_parameter(rclcpp::Parameter("speed_factor", 5.0));
  EXPECT_TRUE(result.successful);
  EXPECT_DOUBLE_EQ(sim_node_->get_parameter("speed_factor").as_double(), 5.0);

  // Reject non-positive
  result = sim_node_->set_parameter(rclcpp::Parameter("speed_factor", 0.0));
  EXPECT_FALSE(result.successful);

  result = sim_node_->set_parameter(rclcpp::Parameter("speed_factor", -1.0));
  EXPECT_FALSE(result.successful);

  // Should still be last valid value
  EXPECT_DOUBLE_EQ(sim_node_->get_parameter("speed_factor").as_double(), 5.0);
}
