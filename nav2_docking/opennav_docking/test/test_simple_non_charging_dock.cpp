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

#include <thread>
#include <chrono>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "opennav_docking/simple_non_charging_dock.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

// Testing the simple non-charging dock plugin

namespace opennav_docking
{

  class SimpleNonChargingDockShim : public opennav_docking::SimpleNonChargingDock
  {
  public:
    SimpleNonChargingDockShim()
    : opennav_docking::SimpleNonChargingDock() {}
  
    using DetectorState = opennav_docking::SimpleChargingDock::DetectorState;
  
    std::vector<std::string> getStallJointNames()
    {
      return stall_joint_names_;
    }
  
    bool detectorIsOn() const
    {
      return detector_state_ == DetectorState::ON;
    }
  
    // Exposes detector_state_ for testing purposes
    DetectorState getDetectorState() const {
      return detector_state_;
    }
  };

TEST(SimpleNonChargingDockTests, ObjectLifecycle)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));

  auto dock = std::make_unique<opennav_docking::SimpleNonChargingDock>();
  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  // Check initial states
  EXPECT_THROW(dock->isCharging(), std::runtime_error);
  EXPECT_THROW(dock->disableCharging(), std::runtime_error);
  EXPECT_THROW(dock->hasStoppedCharging(), std::runtime_error);
  EXPECT_FALSE(dock->isCharger());

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleNonChargingDockTests, DetectorStateTransitions)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true)); // Needed for state changes
  node->declare_parameter("my_dock.subscribe_toggle", rclcpp::ParameterValue(true)); // To allow state changes

  auto dock = std::make_unique<opennav_docking::SimpleNonChargingDockShim>();
  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  EXPECT_TRUE(dock->getDetectorState() == opennav_docking::SimpleNonChargingDockShim::DetectorState::OFF);
  dock->startDetectionProcess();
  EXPECT_TRUE(dock->getDetectorState() == opennav_docking::SimpleNonChargingDockShim::DetectorState::ON);
  dock->stopDetectionProcess();
  EXPECT_TRUE(dock->getDetectorState() == opennav_docking::SimpleNonChargingDockShim::DetectorState::OFF);

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

// Test to verify subscription behavior when subscribe_toggle is false.
// The subscription should be created during configure and remain active
// regardless of start/stopDetectionProcess calls.
TEST(SimpleNonChargingDockTests, SubscriptionAlwaysOnWhenToggleIsFalse)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_sub_always_on_noncharging");
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));
  node->declare_parameter("my_dock.subscribe_toggle", rclcpp::ParameterValue(false));
  node->declare_parameter("my_dock.detector_service_name", rclcpp::ParameterValue("")); // Disable service calls

  auto dock_shim = std::make_unique<opennav_docking::SimpleNonChargingDockShim>();

  // Configure plugin: subscription to detected_dock_pose should be established.
  ASSERT_NO_THROW(dock_shim->configure(node, "my_dock", nullptr));

  // Activate plugin. Initial detector state should be OFF.
  ASSERT_NO_THROW(dock_shim->activate());
  EXPECT_TRUE(dock_shim->getDetectorState() == opennav_docking::SimpleNonChargingDockShim::DetectorState::OFF);

  // Call startDetectionProcess. Detector state should transition to ON.
  // The existing subscription should not be affected.
  ASSERT_NO_THROW(dock_shim->startDetectionProcess());
  EXPECT_TRUE(dock_shim->getDetectorState() == opennav_docking::SimpleNonChargingDockShim::DetectorState::ON);

  // Publish a pose and verify getRefinedPose receives it.
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("detected_dock_pose", rclcpp::SensorDataQoS());
  ASSERT_NO_THROW(pub->on_activate());

  geometry_msgs::msg::PoseStamped detected_msg;
  detected_msg.header.stamp = node->now();
  detected_msg.header.frame_id = "test_frame";
  detected_msg.pose.position.x = 1.23; // Expecting 1.03 refined (1.23 - 0.2 default translation_x)
  detected_msg.pose.orientation.w = 1.0;
  pub->publish(detected_msg);

  rclcpp::spin_some(node->get_node_base_interface());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(node->get_node_base_interface());

  geometry_msgs::msg::PoseStamped refined_pose_msg;
  refined_pose_msg.header.frame_id = "test_frame";
  EXPECT_TRUE(dock_shim->getRefinedPose(refined_pose_msg, ""));
  EXPECT_NEAR(refined_pose_msg.pose.position.x, 1.03, 1e-5);

  // Call stopDetectionProcess. Detector state should transition to OFF.
  // The subscription should remain active.
  ASSERT_NO_THROW(dock_shim->stopDetectionProcess());
  EXPECT_TRUE(dock_shim->getDetectorState() == opennav_docking::SimpleNonChargingDockShim::DetectorState::OFF);

  // Publish another pose. getRefinedPose should still process it due to persistent subscription.
  detected_msg.header.stamp = node->now();
  detected_msg.pose.position.x = 4.76; // Expecting 4.56
  pub->publish(detected_msg);
  rclcpp::spin_some(node->get_node_base_interface());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(node->get_node_base_interface());

  EXPECT_TRUE(dock_shim->getRefinedPose(refined_pose_msg, ""));
  EXPECT_NEAR(refined_pose_msg.pose.position.x, 4.56, 1e-5);

  ASSERT_NO_THROW(dock_shim->deactivate());
  ASSERT_NO_THROW(dock_shim->cleanup());
}

// Test behavior when external detection is not used.
TEST(SimpleNonChargingDockTests, UseExternalDetectionPoseFalse_IgnoresDetector)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_no_ext_detect_noncharging");
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(false));
  node->declare_parameter("my_dock.detector_service_name", rclcpp::ParameterValue("should_not_be_called_service"));
  node->declare_parameter("my_dock.subscribe_toggle", rclcpp::ParameterValue(true));

  auto dock_shim = std::make_unique<opennav_docking::SimpleNonChargingDockShim>();
  ASSERT_NO_THROW(dock_shim->configure(node, "my_dock", nullptr));
  ASSERT_NO_THROW(dock_shim->activate());

  EXPECT_TRUE(dock_shim->getDetectorState() == opennav_docking::SimpleNonChargingDockShim::DetectorState::OFF);
  ASSERT_NO_THROW(dock_shim->startDetectionProcess());
  EXPECT_TRUE(dock_shim->getDetectorState() == opennav_docking::SimpleNonChargingDockShim::DetectorState::ON);

  geometry_msgs::msg::PoseStamped pose_from_db, refined_pose;
  pose_from_db.header.frame_id = "map";
  pose_from_db.pose.position.x = 10.0;
  pose_from_db.pose.orientation.w = 1.0;

  refined_pose = pose_from_db;
  EXPECT_TRUE(dock_shim->getRefinedPose(refined_pose, ""));
  EXPECT_DOUBLE_EQ(refined_pose.pose.position.x, pose_from_db.pose.position.x);
  EXPECT_DOUBLE_EQ(refined_pose.pose.orientation.w, pose_from_db.pose.orientation.w);

  ASSERT_NO_THROW(dock_shim->stopDetectionProcess());
  EXPECT_TRUE(dock_shim->getDetectorState() == opennav_docking::SimpleNonChargingDockShim::DetectorState::OFF);

  ASSERT_NO_THROW(dock_shim->deactivate());
  ASSERT_NO_THROW(dock_shim->cleanup());
}

TEST(SimpleNonChargingDockTests, StallDetection)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto pub = node->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::QoS(1));
  pub->on_activate();
  node->declare_parameter("my_dock.use_stall_detection", rclcpp::ParameterValue(true));
  node->declare_parameter("my_dock.stall_joint_names", rclcpp::PARAMETER_STRING_ARRAY);
  node->declare_parameter("my_dock.stall_velocity_threshold", rclcpp::ParameterValue(0.1));
  node->declare_parameter("my_dock.stall_effort_threshold", rclcpp::ParameterValue(5.0));

  auto dock = std::make_unique<opennav_docking::SimpleNonChargingDockShim>();
  dock->configure(node, "my_dock", nullptr);
  // Check that the joint names are empty, showing the error
  EXPECT_TRUE(dock->getStallJointNames().empty());
  dock->cleanup();

  // Now set the joint names
  std::vector<std::string> names = {"left_motor", "right_motor"};
  node->set_parameter(
    rclcpp::Parameter("my_dock.stall_joint_names", rclcpp::ParameterValue(names)));
  dock->configure(node, "my_dock", nullptr);
  EXPECT_EQ(dock->getStallJointNames(), names);

  dock->activate();
  geometry_msgs::msg::PoseStamped pose;
  EXPECT_TRUE(dock->getRefinedPose(pose, ""));

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

TEST(SimpleNonChargingDockTests, StagingPose)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto dock = std::make_unique<opennav_docking::SimpleNonChargingDock>();

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

TEST(SimpleNonChargingDockTests, StagingPoseWithYawOffset)
{
  // Override the parameter default
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      {"my_dock.staging_yaw_offset", 3.14},
    }
  );

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test", options);
  auto dock = std::make_unique<opennav_docking::SimpleNonChargingDock>();

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

TEST(SimpleNonChargingDockTests, RefinedPoseTest)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "detected_dock_pose", rclcpp::QoS(1));
  pub->on_activate();
  auto dock = std::make_unique<opennav_docking::SimpleNonChargingDock>();

  dock->configure(node, "my_dock", nullptr);
  dock->activate();

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

  pose.header.frame_id = "my_frame";
  EXPECT_TRUE(dock->getRefinedPose(pose, ""));
  EXPECT_NEAR(pose.pose.position.x, 0.1, 0.01);
  EXPECT_NEAR(pose.pose.position.y, -0.3, 0.01);  // Applies external_detection_translation_x, +0.2

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleNonChargingDockTests, RefinedPoseNotTransform)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "detected_dock_pose", rclcpp::QoS(1));
  pub->on_activate();
  auto dock = std::make_unique<opennav_docking::SimpleNonChargingDock>();

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

TEST(SimpleNonChargingDockTests, IsDockedTransformException)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "detected_dock_pose", rclcpp::QoS(1));
  pub->on_activate();
  auto dock = std::make_unique<opennav_docking::SimpleNonChargingDock>();

  // Create the TF
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_buffer->setUsingDedicatedThread(true);

  dock->configure(node, "my_dock", tf_buffer);
  dock->activate();

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
  dock->cleanup();
  dock.reset();
  tf_buffer.reset();
}

TEST(SimpleNonChargingDockTests, GetDockDirection)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  node->declare_parameter("my_dock.dock_direction", rclcpp::ParameterValue("forward"));

  auto dock = std::make_unique<opennav_docking::SimpleNonChargingDock>();
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
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");

  // Case 1: Direction to BACKWARD and rotate_to_dock to true
  node->declare_parameter("my_dock.dock_direction", rclcpp::ParameterValue("backward"));
  node->declare_parameter("my_dock.rotate_to_dock", rclcpp::ParameterValue(true));

  auto dock = std::make_unique<opennav_docking::SimpleNonChargingDock>();
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

TEST(SimpleNonChargingDockTests, DetectorLifecycle)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");

  // Test with detector service configured
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));
  node->declare_parameter("my_dock.detector_service_name",
      rclcpp::ParameterValue("test_detector_service"));
  node->declare_parameter("my_dock.subscribe_toggle", rclcpp::ParameterValue(true));

  auto dock = std::make_unique<opennav_docking::SimpleNonChargingDock>();
  dock->configure(node, "my_dock", nullptr);

  dock->activate();
  dock->startDetectionProcess();
  dock->stopDetectionProcess();
  dock->deactivate();

  dock->cleanup();
  dock.reset();
}

TEST(SimpleNonChargingDockTests, DetectorAutoStart)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "detected_dock_pose", rclcpp::QoS(1));
  pub->on_activate();

  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));
  node->declare_parameter("my_dock.detector_service_name", rclcpp::ParameterValue(""));
  node->declare_parameter("my_dock.subscribe_toggle", rclcpp::ParameterValue(true));

  auto dock = std::make_unique<opennav_docking::SimpleNonChargingDock>();
  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  dock->startDetectionProcess();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "test_frame";

  // Should return false (no detection yet)
  EXPECT_FALSE(dock->getRefinedPose(pose, ""));

  // Publish a detection
  geometry_msgs::msg::PoseStamped detected_pose;
  detected_pose.header.stamp = node->now();
  detected_pose.header.frame_id = "test_frame";
  detected_pose.pose.position.x = 1.0;
  detected_pose.pose.position.y = 2.0;
  pub->publish(detected_pose);

  rclcpp::spin_some(node->get_node_base_interface());

  // Now should get the detected pose
  EXPECT_TRUE(dock->getRefinedPose(pose, ""));

  dock->stopDetectionProcess();
  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleNonChargingDockTests, ServiceDetectorControl)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");

  // Configure with detector service FIRST
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));
  node->declare_parameter("my_dock.detector_service_name",
      rclcpp::ParameterValue("test_detector_service"));
  node->declare_parameter("my_dock.detector_service_timeout", rclcpp::ParameterValue(1.0));

  // Create a mock service BEFORE configuring the dock
  bool service_called = false;
  auto service = node->create_service<std_srvs::srv::Trigger>(
    "test_detector_service",
    [&service_called](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      service_called = true;
      response->success = true;
    });

  // Let the service be ready
  rclcpp::spin_some(node->get_node_base_interface());

  auto dock = std::make_unique<opennav_docking::SimpleNonChargingDock>();
  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  dock->startDetectionProcess();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "test_frame";

  // Wait for async service call to be processed
  auto start_time = std::chrono::steady_clock::now();
  while (!service_called &&
    std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2))
  {
    rclcpp::spin_some(node->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Service should have been called
  EXPECT_TRUE(service_called);
  dock->stopDetectionProcess();
  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleNonChargingDockTests, DetectorServiceTimeout)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");

  // Configure with a service that doesn't exist
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));
  node->declare_parameter("my_dock.detector_service_name",
      rclcpp::ParameterValue("non_existent_service"));
  node->declare_parameter("my_dock.detector_service_timeout",
                        rclcpp::ParameterValue(0.1));  // Short timeout

  auto dock = std::make_unique<opennav_docking::SimpleNonChargingDock>();
  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  // Should handle timeout gracefully and continue with subscription only
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "test_frame";
  EXPECT_FALSE(dock->getRefinedPose(pose, ""));  // No detection available

  dock->stopDetectionProcess();
  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleNonChargingDockTests, NoDetectorService)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");

  // Configure without detector service (default behavior)
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));
  node->declare_parameter("my_dock.detector_service_name", rclcpp::ParameterValue(""));
  node->declare_parameter("my_dock.subscribe_toggle", rclcpp::ParameterValue(false));

  auto dock = std::make_unique<opennav_docking::SimpleNonChargingDock>();
  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  // Should work without service or subscription
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "test_frame";
  EXPECT_FALSE(dock->getRefinedPose(pose, ""));  // No detection available

  dock->stopDetectionProcess();
  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

// Test that deactivate stops the detector if it was running.
TEST(SimpleNonChargingDockTests, DeactivateStopsDetector)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_deactivate_stops_charging");
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));
  node->declare_parameter("my_dock.subscribe_toggle", rclcpp::ParameterValue(true));

  auto dock_shim = std::make_unique<opennav_docking::SimpleNonChargingDockShim>();
  ASSERT_NO_THROW(dock_shim->configure(node, "my_dock", nullptr));
  ASSERT_NO_THROW(dock_shim->activate());

  ASSERT_NO_THROW(dock_shim->startDetectionProcess());
  EXPECT_TRUE(dock_shim->getDetectorState() == opennav_docking::SimpleNonChargingDockShim::DetectorState::ON);

  ASSERT_NO_THROW(dock_shim->deactivate());
  EXPECT_TRUE(dock_shim->getDetectorState() == opennav_docking::SimpleNonChargingDockShim::DetectorState::OFF);

  ASSERT_NO_THROW(dock_shim->cleanup());
  EXPECT_TRUE(dock_shim->getDetectorState() == opennav_docking::SimpleNonChargingDockShim::DetectorState::OFF);
}

// Test that cleanup stops the detector if it was running and deactivate was skipped.
TEST(SimpleNonChargingDockTests, CleanupStopsDetectorIfDeactivateSkipped)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_cleanup_stops_charging");
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));
  node->declare_parameter("my_dock.subscribe_toggle", rclcpp::ParameterValue(true));

  auto dock_shim = std::make_unique<opennav_docking::SimpleNonChargingDockShim>();
  ASSERT_NO_THROW(dock_shim->configure(node, "my_dock", nullptr));
  ASSERT_NO_THROW(dock_shim->activate());

  ASSERT_NO_THROW(dock_shim->startDetectionProcess());
  EXPECT_TRUE(dock_shim->getDetectorState() == opennav_docking::SimpleNonChargingDockShim::DetectorState::ON);

  ASSERT_NO_THROW(dock_shim->cleanup());
  EXPECT_TRUE(dock_shim->getDetectorState() == opennav_docking::SimpleNonChargingDockShim::DetectorState::OFF);
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
