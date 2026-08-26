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
#include <vector>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "opennav_docking/graceful_controller.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

// The control law itself lives in nav2_graceful_controller, which has over 98% test coverage.
// These tests cover the plugin wrapper: parameter namespacing, frame resolution, and the
// trajectory handling introduced by the opennav_docking::ControllerBase interface.

namespace opennav_docking
{

/// @brief Build a path in the given frame from a list of (x, y, yaw) triples.
nav_msgs::msg::Path makePath(
  const std::string & frame, const std::vector<std::array<double, 3>> & points)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  for (const auto & point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.pose.position.x = point[0];
    pose.pose.position.y = point[1];
    pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(point[2]);
    path.poses.push_back(pose);
  }
  return path;
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
    node, "c.base_frame", rclcpp::ParameterValue(std::string("base_link")));
  nav2::declare_parameter_if_not_declared(
    node, "c.fixed_frame", rclcpp::ParameterValue(std::string("base_link")));

  auto controller = loader.createSharedInstance("opennav_docking::GracefulController");
  controller->configure(node, "c", tf);
  EXPECT_EQ(controller->getName(), "c");

  controller->setTrajectory(makePath("base_link", {{1.0, 0.0, 0.0}}));
  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::Twist cmd;
  EXPECT_TRUE(controller->computeVelocityCommands(robot_pose, geometry_msgs::msg::Twist(), 0.1,
    cmd));
  EXPECT_GT(cmd.linear.x, 0.0);

  controller->reset();
  controller->activate();
  controller->deactivate();
  controller->cleanup();
  controller.reset();
}

TEST(GracefulControllerTests, ParameterNamespacesAreIsolated)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  auto tf = nav2::create_transform_buffer(node);
  tf->setUsingDedicatedThread(true);
  setTransform(tf, "odom", "base_link", 0.0, 0.0);

  nav2::declare_parameter_if_not_declared(
    node, "fixed_frame", rclcpp::ParameterValue(std::string("odom")));
  auto c1 = makeController(node, tf, "c1");
  auto c2 = makeController(node, tf, "c2");

  // Slow c1 down; c2 shares the node but must keep its defaults
  node->set_parameters(
    {rclcpp::Parameter("c1.v_linear_min", 0.01), rclcpp::Parameter("c1.v_linear_max", 0.05)});

  auto trajectory = makePath("base_link", {{2.0, 0.0, 0.0}});
  c1->setTrajectory(trajectory);
  c2->setTrajectory(trajectory);

  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::Twist cmd1, cmd2;
  EXPECT_TRUE(c1->computeVelocityCommands(robot_pose, geometry_msgs::msg::Twist(), 0.1, cmd1));
  EXPECT_TRUE(c2->computeVelocityCommands(robot_pose, geometry_msgs::msg::Twist(), 0.1, cmd2));

  EXPECT_NEAR(cmd1.linear.x, 0.05, 1e-6);
  EXPECT_NEAR(cmd2.linear.x, 0.25, 1e-6);
}

TEST(GracefulControllerTests, FramePrecedencePrefersInstanceOverNode)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  auto tf = nav2::create_transform_buffer(node);
  tf->setUsingDedicatedThread(true);
  setTransform(tf, "odom", "my_base", 0.0, 0.0);

  // The node-level frames name something that does not exist in the TF tree
  nav2::declare_parameter_if_not_declared(
    node, "fixed_frame", rclcpp::ParameterValue(std::string("bogus_fixed")));
  nav2::declare_parameter_if_not_declared(
    node, "base_frame", rclcpp::ParameterValue(std::string("bogus_base")));
  nav2::declare_parameter_if_not_declared(
    node, "c.fixed_frame", rclcpp::ParameterValue(std::string("odom")));
  nav2::declare_parameter_if_not_declared(
    node, "c.base_frame", rclcpp::ParameterValue(std::string("my_base")));

  auto controller = makeController(node, tf, "c");
  controller->setTrajectory(makePath("odom", {{1.0, 0.0, 0.0}}));

  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::Twist cmd;
  EXPECT_TRUE(controller->computeVelocityCommands(robot_pose, geometry_msgs::msg::Twist(), 0.1,
    cmd));
  EXPECT_GT(cmd.linear.x, 0.0);
}

TEST(GracefulControllerTests, FramePrecedenceFallsBackToNode)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  auto tf = nav2::create_transform_buffer(node);
  tf->setUsingDedicatedThread(true);
  setTransform(tf, "odom", "my_base", 0.0, 0.0);

  // No per-instance override: the server's frames must be picked up
  nav2::declare_parameter_if_not_declared(
    node, "fixed_frame", rclcpp::ParameterValue(std::string("odom")));
  nav2::declare_parameter_if_not_declared(
    node, "base_frame", rclcpp::ParameterValue(std::string("my_base")));

  auto controller = makeController(node, tf, "c");
  controller->setTrajectory(makePath("odom", {{1.0, 0.0, 0.0}}));

  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::Twist cmd;
  EXPECT_TRUE(controller->computeVelocityCommands(robot_pose, geometry_msgs::msg::Twist(), 0.1,
    cmd));
  EXPECT_GT(cmd.linear.x, 0.0);
}

TEST(GracefulControllerTests, TrajectoryIsTransformedIntoBaseFrame)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  auto tf = nav2::create_transform_buffer(node);
  tf->setUsingDedicatedThread(true);

  // The robot sits 1m to the left of the odom origin, facing along +x
  setTransform(tf, "odom", "base_link", 0.0, 1.0);
  nav2::declare_parameter_if_not_declared(
    node, "fixed_frame", rclcpp::ParameterValue(std::string("odom")));
  auto controller = makeController(node, tf, "c");

  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::Twist cmd;

  // A target on the odom x-axis lies 1m to the robot's right, so the robot must steer right.
  // Reading the odom coordinates literally would place it straight ahead and steer nowhere.
  controller->setTrajectory(makePath("odom", {{1.0, 0.0, 0.0}}));
  EXPECT_TRUE(controller->computeVelocityCommands(robot_pose, geometry_msgs::msg::Twist(), 0.1,
    cmd));
  EXPECT_GT(cmd.linear.x, 0.0);
  EXPECT_LT(cmd.angular.z, -1e-3);

  // A target on the robot's own row is straight ahead, so it drives without steering
  controller->setTrajectory(makePath("odom", {{2.0, 1.0, 0.0}}));
  EXPECT_TRUE(controller->computeVelocityCommands(robot_pose, geometry_msgs::msg::Twist(), 0.1,
    cmd));
  EXPECT_GT(cmd.linear.x, 0.0);
  EXPECT_NEAR(cmd.angular.z, 0.0, 1e-6);
}

TEST(GracefulControllerTests, LastPoseOfTrajectoryIsTheTarget)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  auto tf = nav2::create_transform_buffer(node);
  tf->setUsingDedicatedThread(true);
  setTransform(tf, "odom", "base_link", 0.0, 0.0);
  nav2::declare_parameter_if_not_declared(
    node, "fixed_frame", rclcpp::ParameterValue(std::string("odom")));
  auto controller = makeController(node, tf, "c");

  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::Twist single, multi;

  controller->setTrajectory(makePath("base_link", {{1.0, 0.5, 0.0}}));
  EXPECT_TRUE(controller->computeVelocityCommands(robot_pose, geometry_msgs::msg::Twist(), 0.1,
    single));

  controller->setTrajectory(makePath("base_link", {{5.0, -3.0, 1.0}, {1.0, 0.5, 0.0}}));
  EXPECT_TRUE(controller->computeVelocityCommands(robot_pose, geometry_msgs::msg::Twist(), 0.1,
    multi));

  EXPECT_EQ(single, multi);
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

  opennav_docking::TrajectoryOptions options;
  options.reverse = true;
  controller->setTrajectory(makePath("base_link", {{-1.0, 0.0, M_PI}}), options);

  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::Twist cmd;
  EXPECT_TRUE(controller->computeVelocityCommands(robot_pose, geometry_msgs::msg::Twist(), 0.1,
    cmd));
  EXPECT_LT(cmd.linear.x, 0.0);
}

TEST(GracefulControllerTests, InvalidTrajectoriesFailWithoutCrashing)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  auto tf = nav2::create_transform_buffer(node);
  tf->setUsingDedicatedThread(true);
  setTransform(tf, "odom", "base_link", 0.0, 0.0);
  nav2::declare_parameter_if_not_declared(
    node, "fixed_frame", rclcpp::ParameterValue(std::string("odom")));
  auto controller = makeController(node, tf, "c");

  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::Twist cmd, zero;

  // Never given a trajectory at all
  EXPECT_FALSE(controller->computeVelocityCommands(robot_pose, geometry_msgs::msg::Twist(), 0.1,
    cmd));
  EXPECT_EQ(cmd, zero);

  // An empty path
  controller->setTrajectory(makePath("base_link", {}));
  EXPECT_FALSE(controller->computeVelocityCommands(robot_pose, geometry_msgs::msg::Twist(), 0.1,
    cmd));
  EXPECT_EQ(cmd, zero);

  // A path with no frame at all
  controller->setTrajectory(makePath("", {{1.0, 0.0, 0.0}}));
  EXPECT_FALSE(controller->computeVelocityCommands(robot_pose, geometry_msgs::msg::Twist(), 0.1,
    cmd));
  EXPECT_EQ(cmd, zero);

  // A path in a frame that is not in the TF tree
  controller->setTrajectory(makePath("nowhere", {{1.0, 0.0, 0.0}}));
  EXPECT_FALSE(controller->computeVelocityCommands(robot_pose, geometry_msgs::msg::Twist(), 0.1,
    cmd));
  EXPECT_EQ(cmd, zero);
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

TEST(GracefulControllerTests, TrajectoryPublishingIsOffByDefault)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  auto tf = nav2::create_transform_buffer(node);
  tf->setUsingDedicatedThread(true);
  nav2::declare_parameter_if_not_declared(
    node, "c.use_collision_detection", rclcpp::ParameterValue(false));
  setTransform(tf, "odom", "base_link", 0.0, 0.0);

  auto controller = std::make_shared<TestableGracefulController>();
  controller->configure(node, "c", tf);
  EXPECT_FALSE(controller->hasTrajectoryPublisher());

  // Activation, command computation and deactivation must all tolerate the absent publisher.
  controller->activate();
  controller->setTrajectory(makePath("base_link", {{1.0, -1.0, 0.0}}));
  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::Twist cmd;
  EXPECT_TRUE(
    controller->computeVelocityCommands(robot_pose, geometry_msgs::msg::Twist(), 0.1, cmd));
  EXPECT_GT(cmd.linear.x, 0.0);
  controller->deactivate();
  controller->cleanup();
}

TEST(GracefulControllerTests, TrajectoryPublishingIsEnabledByTheFlag)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  auto tf = nav2::create_transform_buffer(node);
  tf->setUsingDedicatedThread(true);
  nav2::declare_parameter_if_not_declared(
    node, "c.use_collision_detection", rclcpp::ParameterValue(false));
  nav2::declare_parameter_if_not_declared(
    node, "c.publish_trajectory", rclcpp::ParameterValue(true));
  setTransform(tf, "odom", "base_link", 0.0, 0.0);

  auto controller = std::make_shared<TestableGracefulController>();
  controller->configure(node, "c", tf);
  EXPECT_TRUE(controller->hasTrajectoryPublisher());

  controller->activate();
  controller->setTrajectory(makePath("base_link", {{1.0, -1.0, 0.0}}));
  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::Twist cmd;
  EXPECT_TRUE(
    controller->computeVelocityCommands(robot_pose, geometry_msgs::msg::Twist(), 0.1, cmd));
  EXPECT_GT(cmd.linear.x, 0.0);
  controller->deactivate();
  controller->cleanup();
  EXPECT_FALSE(controller->hasTrajectoryPublisher());
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
