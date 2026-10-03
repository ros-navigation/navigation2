// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
// Copyright (c) 2026 Karinca Robotics
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

#include "gtest/gtest.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "opennav_docking/controller_plugins/graceful_controller.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"


namespace opennav_docking
{

/// @brief Build a target pose in the robot's base frame.
geometry_msgs::msg::Pose makeTarget(double x, double y, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  return pose;
}

/// @brief Insert a static transform directly into the buffer, so no spinning is required.
void setTransform(
  const nav2::TransformBuffer::SharedPtr & tf,
  const std::string & parent, const std::string & child, double x, double y)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = parent;
  transform.child_frame_id = child;
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.rotation.w = 1.0;
  tf->setTransform(transform, "test", true);
}

/// @brief Exposes protected state so the trajectory publisher can be inspected directly.
class TestableGracefulController : public GracefulController
{
public:
  bool hasTrajectoryPublisher() const {return trajectory_pub_ != nullptr;}
};

/// @brief Configure a controller instance with collision detection disabled.
std::shared_ptr<GracefulController> makeController(
  const nav2::LifecycleNode::SharedPtr & node, const nav2::TransformBuffer::SharedPtr & tf,
  const std::string & name)
{
  nav2::declare_parameter_if_not_declared(
    node, name + ".use_collision_detection", rclcpp::ParameterValue(false));
  auto controller = std::make_shared<GracefulController>();
  controller->configure(node, name, tf);
  return controller;
}

TEST(GracefulControllerTests, PluginIsDiscoverable)
{
  pluginlib::ClassLoader<opennav_docking::ControllerBase> loader(
    "opennav_docking", "opennav_docking::ControllerBase");
  EXPECT_TRUE(loader.isClassAvailable("opennav_docking::GracefulController"));

  auto node = std::make_shared<nav2::LifecycleNode>("test");
  auto tf = nav2::create_transform_buffer(node);
  tf->setUsingDedicatedThread(true);
  nav2::declare_parameter_if_not_declared(
    node, "c.use_collision_detection", rclcpp::ParameterValue(false));
  nav2::declare_parameter_if_not_declared(
    node, "base_frame", rclcpp::ParameterValue(std::string("base_link")));
  nav2::declare_parameter_if_not_declared(
    node, "fixed_frame", rclcpp::ParameterValue(std::string("base_link")));

  auto controller = loader.createSharedInstance("opennav_docking::GracefulController");
  controller->configure(node, "c", tf);
  EXPECT_EQ(controller->getName(), "c");

  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::Twist cmd;
  EXPECT_TRUE(controller->computeVelocityCommands(
    robot_pose, geometry_msgs::msg::Twist(), makeTarget(1.0, 0.0, 0.0), {}, 0.1,
    cmd));
  EXPECT_GT(cmd.linear.x, 0.0);

  controller->reset();
  controller->activate();
  controller->deactivate();
  controller->cleanup();
  controller.reset();
}

TEST(GracefulControllerTests, UsesServerFrames)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  auto tf = nav2::create_transform_buffer(node);
  tf->setUsingDedicatedThread(true);
  setTransform(tf, "odom", "my_base", 0.0, 0.0);

  // The controller must pick up the server's frames
  nav2::declare_parameter_if_not_declared(
    node, "fixed_frame", rclcpp::ParameterValue(std::string("odom")));
  nav2::declare_parameter_if_not_declared(
    node, "base_frame", rclcpp::ParameterValue(std::string("my_base")));

  auto controller = makeController(node, tf, "c");
  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::Twist cmd;
  EXPECT_TRUE(controller->computeVelocityCommands(
    robot_pose, geometry_msgs::msg::Twist(), makeTarget(1.0, 0.0, 0.0), {}, 0.1,
    cmd));
  EXPECT_GT(cmd.linear.x, 0.0);
}

TEST(GracefulControllerTests, ReverseDrivesBackwards)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  auto tf = nav2::create_transform_buffer(node);
  tf->setUsingDedicatedThread(true);
  setTransform(tf, "odom", "base_link", 0.0, 0.0);
  nav2::declare_parameter_if_not_declared(
    node, "fixed_frame", rclcpp::ParameterValue(std::string("odom")));
  auto controller = makeController(node, tf, "c");

  opennav_docking::DockingOptions options;
  options.reverse = true;
  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::Twist cmd;
  EXPECT_TRUE(controller->computeVelocityCommands(
    robot_pose, geometry_msgs::msg::Twist(), makeTarget(-1.0, 0.0, M_PI), options,
    0.1, cmd));
  EXPECT_LT(cmd.linear.x, 0.0);
}

TEST(GracefulControllerTests, MissingTransformFails)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  auto tf = nav2::create_transform_buffer(node);
  tf->setUsingDedicatedThread(true);
  nav2::declare_parameter_if_not_declared(
    node, "fixed_frame", rclcpp::ParameterValue(std::string("nowhere")));
  auto controller = makeController(node, tf, "c");

  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::Twist cmd;
  EXPECT_FALSE(controller->computeVelocityCommands(
    robot_pose, geometry_msgs::msg::Twist(), makeTarget(1.0, 0.0, 0.0), {}, 0.1, cmd));
}

TEST(GracefulControllerTests, RotateToHeadingUsesInstanceParameters)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  auto tf = nav2::create_transform_buffer(node);
  tf->setUsingDedicatedThread(true);
  nav2::declare_parameter_if_not_declared(
    node, "c.rotate_to_heading_angular_vel", rclcpp::ParameterValue(0.5));
  auto controller = makeController(node, tf, "c");

  geometry_msgs::msg::Twist current_velocity;
  auto cmd = controller->computeRotateToHeadingCommand(1.0, current_velocity, 0.1);
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
  EXPECT_GT(cmd.angular.z, 0.0);
  EXPECT_LE(cmd.angular.z, 0.5);
}

}  // namespace opennav_docking

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
