// Copyright (c) 2024 Open Navigation LLC
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

#include <chrono>
#include "gtest/gtest.h"
#include "std_srvs/srv/trigger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "opennav_docking/simple_charging_dock.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

// Testing the simple charging dock plugin

using namespace std::chrono_literals;

namespace opennav_docking
{

class SimpleChargingDockShim : public opennav_docking::SimpleChargingDock
{
public:
  SimpleChargingDockShim()
  : opennav_docking::SimpleChargingDock() {}

  std::vector<std::string> getStallJointNames()
  {
    return stall_joint_names_;
  }
};

class SimpleChargingDockTestable : public opennav_docking::SimpleChargingDock
{
public:
  using opennav_docking::SimpleChargingDock::SimpleChargingDock;

   // Expose detector state for test verification
  bool isDetectorActive() const
  {
    return detector_state_ == DetectorState::ON;
  }
};

TEST(SimpleChargingDockTests, ObjectLifecycle)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));

  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();
  dock->configure(node, "my_dock", nullptr);
  dock->activate();
  EXPECT_TRUE(dock->startDetectionProcess());
  EXPECT_TRUE(dock->stopDetectionProcess());

  // Check initial states
  EXPECT_FALSE(dock->isCharging());
  EXPECT_TRUE(dock->disableCharging());
  EXPECT_TRUE(dock->hasStoppedCharging());
  EXPECT_TRUE(dock->isCharger());

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, BatteryState)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  auto pub = node->create_publisher<sensor_msgs::msg::BatteryState>(
    "battery_state", rclcpp::QoS(1));
  pub->on_activate();
  node->declare_parameter("my_dock.use_battery_state", rclcpp::ParameterValue(true));

  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();

  dock->configure(node, "my_dock", nullptr);
  dock->activate();
  geometry_msgs::msg::PoseStamped pose;
  EXPECT_TRUE(dock->getRefinedPose(pose, ""));

  // Below threshold
  sensor_msgs::msg::BatteryState msg;
  msg.current = 0.3;
  pub->publish(msg);
  rclcpp::Rate r(2);
  r.sleep();
  rclcpp::spin_some(node->get_node_base_interface());

  EXPECT_FALSE(dock->isCharging());
  EXPECT_TRUE(dock->hasStoppedCharging());

  // Above threshold
  sensor_msgs::msg::BatteryState msg2;
  msg2.current = 0.6;
  pub->publish(msg2);
  rclcpp::Rate r1(2);
  r1.sleep();
  rclcpp::spin_some(node->get_node_base_interface());

  EXPECT_TRUE(dock->isCharging());
  EXPECT_FALSE(dock->hasStoppedCharging());

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, StallDetection)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  auto pub = node->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::QoS(1));
  pub->on_activate();
  node->declare_parameter("my_dock.use_stall_detection", rclcpp::ParameterValue(true));
  node->declare_parameter("my_dock.stall_joint_names", rclcpp::PARAMETER_STRING_ARRAY);
  node->declare_parameter("my_dock.stall_velocity_threshold", rclcpp::ParameterValue(0.1));
  node->declare_parameter("my_dock.stall_effort_threshold", rclcpp::ParameterValue(5.0));

  auto dock = std::make_unique<opennav_docking::SimpleChargingDockShim>();
  dock->configure(node, "my_dock", nullptr);
  // Check that the joint names are empty, showing the error
  EXPECT_TRUE(dock->getStallJointNames().empty());
  dock->cleanup();

  // Now set the joint names
  std::vector<std::string> names = {"left_motor", "right_motor"};
  node->set_parameter(
    rclcpp::Parameter("my_dock.stall_joint_names", rclcpp::ParameterValue(names)));
  dock->configure(node, "my_dock", nullptr);
  dock->activate();
  EXPECT_EQ(dock->getStallJointNames(), names);

  // Stopped, but below effort threshold
  sensor_msgs::msg::JointState msg;
  msg.name = {"left_motor", "right_motor", "another_motor"};
  msg.velocity = {0.0, 0.0, 0.0};
  msg.effort = {0.0, 0.0, 0.0};
  pub->publish(msg);
  rclcpp::Rate r(2);
  r.sleep();
  rclcpp::spin_some(node->get_node_base_interface());

  EXPECT_FALSE(dock->isDocked());

  // Moving, with effort
  sensor_msgs::msg::JointState msg2;
  msg2.name = {"left_motor", "right_motor", "another_motor"};
  msg2.velocity = {1.0, 1.0, 0.0};
  msg2.effort = {5.1, -5.1, 0.0};
  pub->publish(msg2);
  rclcpp::Rate r1(2);
  r1.sleep();
  rclcpp::spin_some(node->get_node_base_interface());

  EXPECT_FALSE(dock->isDocked());

  // Stopped, with effort
  sensor_msgs::msg::JointState msg3;
  msg3.name = {"left_motor", "right_motor", "another_motor"};
  msg3.velocity = {0.0, 0.0, 0.0};
  msg3.effort = {5.1, -5.1, 0.0};
  pub->publish(msg3);
  rclcpp::Rate r2(2);
  r2.sleep();
  rclcpp::spin_some(node->get_node_base_interface());

  EXPECT_TRUE(dock->isDocked());

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, StagingPose)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();

  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  geometry_msgs::msg::Pose pose;
  std::string frame = "my_frame";
  auto staging_pose = dock->getStagingPose(pose, frame);
  EXPECT_NEAR(staging_pose.pose.position.x, -0.7, 0.01);
  EXPECT_NEAR(staging_pose.pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(tf2::getYaw(staging_pose.pose.orientation), 0.0, 0.01);
  EXPECT_EQ(staging_pose.header.frame_id, frame);

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, StagingPoseWithYawOffset)
{
  // Override the parameter default
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      {"my_dock.staging_yaw_offset", 3.14},
    }
  );

  auto node = std::make_shared<nav2::LifecycleNode>("test", "", options);
  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();

  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  geometry_msgs::msg::Pose pose;
  std::string frame = "my_frame";
  auto staging_pose = dock->getStagingPose(pose, frame);
  // Pose should be the same as default, but pointing in opposite direction
  EXPECT_NEAR(staging_pose.pose.position.x, -0.7, 0.01);
  EXPECT_NEAR(staging_pose.pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(tf2::getYaw(staging_pose.pose.orientation), 3.14, 0.01);
  EXPECT_EQ(staging_pose.header.frame_id, frame);

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, RefinedPoseTest)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "detected_dock_pose", rclcpp::QoS(1));
  pub->on_activate();
  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();

  dock->configure(node, "my_dock", nullptr);
  dock->activate();
  dock->startDetectionProcess();

  geometry_msgs::msg::PoseStamped pose;

  // Timestamps are outdated; this is after timeout
  EXPECT_FALSE(dock->isDocked());
  EXPECT_FALSE(dock->getRefinedPose(pose, ""));

  geometry_msgs::msg::PoseStamped detected_pose;
  detected_pose.header.stamp = node->now();
  detected_pose.header.frame_id = "my_frame";
  detected_pose.pose.position.x = 0.1;
  detected_pose.pose.position.y = -0.5;
  pub->publish(detected_pose);
  rclcpp::spin_some(node->get_node_base_interface());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  pose.header.frame_id = "my_frame";
  EXPECT_TRUE(dock->getRefinedPose(pose, ""));
  EXPECT_NEAR(pose.pose.position.x, 0.1, 0.01);
  EXPECT_NEAR(pose.pose.position.y, -0.3, 0.01);  // Applies external_detection_translation_x, +0.2

  dock->deactivate();
  dock->stopDetectionProcess();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, RefinedPoseNotTransform)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "detected_dock_pose", rclcpp::QoS(1));
  pub->on_activate();
  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();

  // Create the TF
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_buffer->setUsingDedicatedThread(true);

  dock->configure(node, "my_dock", tf_buffer);
  dock->activate();

  geometry_msgs::msg::PoseStamped detected_pose;
  detected_pose.header.stamp = node->now();
  detected_pose.header.frame_id = "my_frame";
  detected_pose.pose.position.x = 1.0;
  detected_pose.pose.position.y = 1.0;
  pub->publish(detected_pose);
  rclcpp::spin_some(node->get_node_base_interface());

  // Create a pose with a different frame_id
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "other_frame";

  // It can not find a transform between the two frames
  EXPECT_FALSE(dock->getRefinedPose(pose, ""));

  dock->deactivate();
  dock->cleanup();
  dock.reset();
  tf_buffer.reset();
}

TEST(SimpleChargingDockTests, IsDockedTransformException)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "detected_dock_pose", rclcpp::QoS(1));
  pub->on_activate();
  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();

  // Create the TF
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_buffer->setUsingDedicatedThread(true);

  dock->configure(node, "my_dock", tf_buffer);
  dock->activate();
  dock->startDetectionProcess();

  // Create a pose with a different frame_id
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "other_frame";

  // Set a transform between the two frames
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node->now();
  transform.header.frame_id = "my_frame";
  transform.child_frame_id = "other_frame";
  tf_buffer->setTransform(transform, "test", true);

  // First call to getRefinedPose starts detection
  EXPECT_FALSE(dock->getRefinedPose(pose, ""));

  // Now publish the detection after subscription is created
  geometry_msgs::msg::PoseStamped detected_pose;
  detected_pose.header.stamp = node->now();
  detected_pose.header.frame_id = "my_frame";
  detected_pose.pose.position.x = 1.0;
  detected_pose.pose.position.y = 1.0;
  pub->publish(detected_pose);
  rclcpp::spin_some(node->get_node_base_interface());

  // Second call should succeed
  EXPECT_TRUE(dock->getRefinedPose(pose, ""));
  EXPECT_FALSE(dock->isDocked());

  dock->deactivate();
  dock->stopDetectionProcess();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, GetDockDirection)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  node->declare_parameter("my_dock.dock_direction", rclcpp::ParameterValue("forward"));

  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();
  EXPECT_EQ(dock->getDockDirection(), opennav_docking_core::DockDirection::UNKNOWN);
  EXPECT_NO_THROW(dock->configure(node, "my_dock", nullptr));
  EXPECT_EQ(dock->getDockDirection(), opennav_docking_core::DockDirection::FORWARD);
  dock->cleanup();

  // Now set to BACKWARD
  node->set_parameter(
    rclcpp::Parameter("my_dock.dock_direction", rclcpp::ParameterValue("backward")));
  EXPECT_NO_THROW(dock->configure(node, "my_dock", nullptr));
  EXPECT_EQ(dock->getDockDirection(), opennav_docking_core::DockDirection::BACKWARD);
  dock->cleanup();

  // Now set to UNKNOWN
  node->set_parameter(
    rclcpp::Parameter("my_dock.dock_direction", rclcpp::ParameterValue("other")));
  EXPECT_THROW(dock->configure(node, "my_dock", nullptr), std::runtime_error);
  EXPECT_EQ(dock->getDockDirection(), opennav_docking_core::DockDirection::UNKNOWN);

  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, ShouldRotateToDock)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");

  // Case 1: Direction to BACKWARD and rotate_to_dock to true
  node->declare_parameter("my_dock.dock_direction", rclcpp::ParameterValue("backward"));
  node->declare_parameter("my_dock.rotate_to_dock", rclcpp::ParameterValue(true));

  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();
  EXPECT_NO_THROW(dock->configure(node, "my_dock", nullptr));
  EXPECT_EQ(dock->shouldRotateToDock(), true);
  dock->cleanup();

  // Case 2: Direction to BACKWARD and rotate_to_dock to false
  node->set_parameter(
    rclcpp::Parameter("my_dock.rotate_to_dock", rclcpp::ParameterValue(false)));
  EXPECT_NO_THROW(dock->configure(node, "my_dock", nullptr));
  EXPECT_EQ(dock->shouldRotateToDock(), false);

  // Case 3: Direction to FORWARD and rotate_to_dock to true
  node->set_parameter(
    rclcpp::Parameter("my_dock.dock_direction", rclcpp::ParameterValue("forward")));
  node->set_parameter(
    rclcpp::Parameter("my_dock.rotate_to_dock", rclcpp::ParameterValue(true)));
  EXPECT_THROW(dock->configure(node, "my_dock", nullptr), std::runtime_error);
  EXPECT_EQ(dock->shouldRotateToDock(), true);
  dock->cleanup();

  // Case 4: Direction to FORWARD and rotate_to_dock to false
  node->set_parameter(
    rclcpp::Parameter("my_dock.rotate_to_dock", rclcpp::ParameterValue(false)));
  EXPECT_NO_THROW(dock->configure(node, "my_dock", nullptr));
  EXPECT_EQ(dock->shouldRotateToDock(), false);

  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, DetectorLifecycle)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");

  // Test with detector service configured
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));
  node->declare_parameter("my_dock.detector_service_name",
      rclcpp::ParameterValue("test_detector_service"));
  node->declare_parameter("my_dock.subscribe_toggle", rclcpp::ParameterValue(true));

  // Create a mock service to prevent timeout
  bool service_called = false;
  auto service = node->create_service<std_srvs::srv::Trigger>(
    "test_detector_service",
    [&service_called](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      service_called = true;
      response->success = true;
    });

  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();
  dock->configure(node, "my_dock", nullptr);

  dock->activate();
  dock->startDetectionProcess();
  // Spin to process async service call
  auto start_time = std::chrono::steady_clock::now();
  while (!service_called &&
    std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2))
  {
    rclcpp::spin_some(node->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(service_called);
  dock->stopDetectionProcess();
  dock->deactivate();

  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, DetectorServiceConfiguration)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_detector_config");

  // Configure with detector service
  node->declare_parameter("my_dock.use_external_detection_pose", true);
  node->declare_parameter("my_dock.detector_service_name", "detector_service");
  node->declare_parameter("my_dock.detector_service_timeout", 1.0);

  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();
  EXPECT_NO_THROW(dock->configure(node, "my_dock", nullptr));
  EXPECT_NO_THROW(dock->activate());

  EXPECT_NO_THROW(dock->deactivate());
  EXPECT_NO_THROW(dock->cleanup());
}

TEST(SimpleChargingDockTests, SubscriptionCallback)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_subscription_reliable");

  node->declare_parameter("my_dock.use_external_detection_pose", true);
  node->declare_parameter("my_dock.subscribe_toggle", true);

  auto dock = std::make_unique<SimpleChargingDockTestable>();
  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  auto publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "detected_dock_pose", 1);
  // A LifecyclePublisher must be activated to publish.
  publisher->on_activate();

  dock->startDetectionProcess();

  // Wait for the publisher and subscriber to connect.
  int tries = 0;
  while (publisher->get_subscription_count() == 0 && tries++ < 10) {
    rclcpp::spin_some(node->get_node_base_interface());
    std::this_thread::sleep_for(100ms);
  }
  ASSERT_GT(publisher->get_subscription_count(), 0);

  // Publish a message to trigger the subscription callback.
  publisher->publish(geometry_msgs::msg::PoseStamped{});
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(node->get_node_base_interface());

  // Verify the detector state was updated, proving the callback was executed.
  EXPECT_TRUE(dock->isDetectorActive());

  dock->deactivate();
  dock->cleanup();
}

TEST(SimpleChargingDockTests, DetectorServiceTimeout)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_detector_timeout");

  node->declare_parameter("my_dock.use_external_detection_pose", true);
  node->declare_parameter("my_dock.detector_service_name", "slow_service");
  node->declare_parameter("my_dock.detector_service_timeout", 0.1);

  // Create a mock service that never responds in time
  auto mock_service = node->create_service<std_srvs::srv::Trigger>(
    "slow_service",
    [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response>) {
      std::this_thread::sleep_for(200ms);
    });

  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();
  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  // The call to invoke() should timeout and be caught
  EXPECT_NO_THROW(dock->startDetectionProcess());

  dock->deactivate();
  dock->cleanup();
  node->shutdown();
}

TEST(SimpleChargingDockTests, DetectorServiceFailure)
{
  const char * service_name = "charging_dock_slow_service";

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      {"my_dock.use_external_detection_pose", true},
      {"my_dock.detector_service_name", std::string(service_name)},
      // The client will time out after 100ms
      {"my_dock.detector_service_timeout", 0.1}
    });
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_detector_failure", options);

  // Create a service that responds slower than the client's timeout.
  auto slow_service = node->create_service<std_srvs::srv::Trigger>(
    service_name,
    [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response>) {
      std::this_thread::sleep_for(200ms);
    });

  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();
  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  // The invoke() call should time out, but the exception must be caught
  // within the startDetectionProcess method. The test passes if no
  // uncaught exception is thrown.
  EXPECT_NO_THROW(dock->startDetectionProcess());

  dock->deactivate();
  dock->cleanup();
}

TEST(SimpleChargingDockTests, SubscriptionPersistent)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      {"my_dock.use_external_detection_pose", true},
      {"my_dock.subscribe_toggle", false}  // The key parameter to test.
    });
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_sub_persistent", options);

  auto dock = std::make_unique<SimpleChargingDockTestable>();
  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  // The subscription should be active immediately after configuration.
  auto publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "detected_dock_pose", 1);
  publisher->on_activate();

  int tries = 0;
  while (publisher->get_subscription_count() == 0 && tries++ < 10) {
    rclcpp::spin_some(node->get_node_base_interface());
    std::this_thread::sleep_for(100ms);
  }
  ASSERT_GT(publisher->get_subscription_count(), 0);

  publisher->publish(geometry_msgs::msg::PoseStamped{});
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(node->get_node_base_interface());

  // Verify the detector state changed, proving the callback was executed.
  EXPECT_TRUE(dock->isDetectorActive());

  dock->deactivate();
  dock->cleanup();
}

}  // namespace opennav_docking

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
