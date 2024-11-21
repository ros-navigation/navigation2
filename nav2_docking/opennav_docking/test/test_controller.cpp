// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "opennav_docking/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_ros/buffer.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

// Testing the controller at high level; the nav2_graceful_controller
// Where the control law derives has over 98% test coverage

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

namespace opennav_docking
{

class ControllerFixture : public opennav_docking::Controller
{
public:
  ControllerFixture(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, std::shared_ptr<tf2_ros::Buffer> tf,
    std::string fixed_frame, std::string base_frame)
  : Controller(node, tf, fixed_frame, base_frame)
  {
  }

  ~ControllerFixture() = default;

  bool isTrajectoryCollisionFree(
    const geometry_msgs::msg::Pose & target_pose, bool is_docking, bool backward = false)
  {
    return opennav_docking::Controller::isTrajectoryCollisionFree(
      target_pose, is_docking, backward);
  }

  void setCollisionTolerance(double tolerance)
  {
    dock_collision_threshold_ = tolerance;
  }
};

class TestCollisionChecker : public nav2_util::LifecycleNode
{
public:
  explicit TestCollisionChecker(std::string name)
  : LifecycleNode(name)
  {
  }

  ~TestCollisionChecker()
  {
    footprint_pub_.reset();
    costmap_pub_.reset();
  }

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(this->get_logger(), "Configuring");

    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, -5.0, -5.0);

    footprint_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
      "test_footprint", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    costmap_pub_ = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
      shared_from_this(), costmap_.get(), "test_base_frame", "test_costmap", true);

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(this->get_logger(), "Activating");
    costmap_pub_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating");
    costmap_pub_->on_deactivate();
    costmap_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  void publishFootprint(
    const double radius, const double center_x, const double center_y,
    std::string base_frame, const rclcpp::Time & stamp)
  {
    std::unique_ptr<geometry_msgs::msg::PolygonStamped> msg =
      std::make_unique<geometry_msgs::msg::PolygonStamped>();

    msg->header.frame_id = base_frame;
    msg->header.stamp = stamp;

    geometry_msgs::msg::Point32 p;

    p.x = center_x + radius;
    p.y = center_y + radius;
    msg->polygon.points.push_back(p);

    p.x = center_x + radius;
    p.y = center_y - radius;
    msg->polygon.points.push_back(p);

    p.x = center_x - radius;
    p.y = center_y - radius;
    msg->polygon.points.push_back(p);

    p.x = center_x - radius;
    p.y = center_y + radius;
    msg->polygon.points.push_back(p);

    footprint_pub_->publish(std::move(msg));
  }

  void publishCostmap()
  {
    costmap_pub_->publishCostmap();
  }

  geometry_msgs::msg::Pose setPose(double x, double y, double theta)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0;
    pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);
    return pose;
  }

  void setRectangle(
    double width, double height, double center_x, double center_y, unsigned char cost)
  {
    unsigned int mx, my;
    if (!costmap_->worldToMap(center_x, center_y, mx, my)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to convert world coordinates to map coordinates");
      return;
    }

    unsigned int width_cell = static_cast<unsigned int>(width / costmap_->getResolution());
    unsigned int height_cell = static_cast<unsigned int>(height / costmap_->getResolution());

    for (unsigned int i = 0; i < width_cell; ++i) {
      for (unsigned int j = 0; j < height_cell; ++j) {
        costmap_->setCost(mx + i, my + j, cost);
      }
    }
  }

  void clearCostmap()
  {
    if (!costmap_) {
      RCLCPP_ERROR(this->get_logger(), "Costmap is not initialized");
      return;
    }

    unsigned int size_x = costmap_->getSizeInCellsX();
    unsigned int size_y = costmap_->getSizeInCellsY();

    for (unsigned int i = 0; i < size_x; ++i) {
      for (unsigned int j = 0; j < size_y; ++j) {
        costmap_->setCost(i, j, nav2_costmap_2d::FREE_SPACE);
      }
    }
  }

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;

  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DPublisher> costmap_pub_;
};

TEST(ControllerTests, ObjectLifecycle)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model

  // Skip collision detection
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.use_collision_detection", rclcpp::ParameterValue(false));

  auto controller = std::make_unique<opennav_docking::Controller>(
    node, tf, "test_base_frame", "test_base_frame");

  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist cmd_out, cmd_init;
  EXPECT_TRUE(controller->computeVelocityCommand(pose, cmd_out, true));
  EXPECT_NE(cmd_init, cmd_out);
  controller.reset();
}

TEST(ControllerTests, DynamicParameters) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto controller = std::make_unique<opennav_docking::Controller>(
    node, nullptr, "test_base_frame", "test_base_frame");

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  // Set parameters
  auto results = params->set_parameters_atomically(
    {rclcpp::Parameter("controller.k_phi", 1.0),
      rclcpp::Parameter("controller.k_delta", 2.0),
      rclcpp::Parameter("controller.beta", 3.0),
      rclcpp::Parameter("controller.lambda", 4.0),
      rclcpp::Parameter("controller.v_linear_min", 5.0),
      rclcpp::Parameter("controller.v_linear_max", 6.0),
      rclcpp::Parameter("controller.v_angular_max", 7.0),
      rclcpp::Parameter("controller.slowdown_radius", 8.0),
      rclcpp::Parameter("controller.projection_time", 9.0),
      rclcpp::Parameter("controller.simulation_time_step", 10.0),
      rclcpp::Parameter("controller.dock_collision_threshold", 11.0)});

  // Spin
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Check parameters
  EXPECT_EQ(node->get_parameter("controller.k_phi").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("controller.k_delta").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("controller.beta").as_double(), 3.0);
  EXPECT_EQ(node->get_parameter("controller.lambda").as_double(), 4.0);
  EXPECT_EQ(node->get_parameter("controller.v_linear_min").as_double(), 5.0);
  EXPECT_EQ(node->get_parameter("controller.v_linear_max").as_double(), 6.0);
  EXPECT_EQ(node->get_parameter("controller.v_angular_max").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("controller.slowdown_radius").as_double(), 8.0);
  EXPECT_EQ(node->get_parameter("controller.projection_time").as_double(), 9.0);
  EXPECT_EQ(node->get_parameter("controller.simulation_time_step").as_double(), 10.0);
  EXPECT_EQ(node->get_parameter("controller.dock_collision_threshold").as_double(), 11.0);
}

TEST(ControllerTests, TFException)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model

  auto controller = std::make_unique<opennav_docking::ControllerFixture>(
    node, tf, "test_fixed_frame", "test_base_frame");

  geometry_msgs::msg::Pose pose;
  EXPECT_FALSE(controller->isTrajectoryCollisionFree(pose, false));
  controller.reset();
}

TEST(ControllerTests, CollisionCheckerDockForward) {
  auto collision_tester = std::make_shared<TestCollisionChecker>("collision_test");
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model

  nav2_util::declare_parameter_if_not_declared(
    node, "controller.footprint_topic", rclcpp::ParameterValue("test_footprint"));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.costmap_topic", rclcpp::ParameterValue("test_costmap_raw"));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.projection_time", rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.simulation_time_step", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.dock_collision_threshold", rclcpp::ParameterValue(0.3));

  auto controller = std::make_unique<opennav_docking::ControllerFixture>(
    node, tf, "test_base_frame", "test_base_frame");
  collision_tester->configure();
  collision_tester->activate();

  // Set the pose of the dock at 1.75m in front of the robot
  auto dock_pose = collision_tester->setPose(1.75, 0.0, 0.0);

  // Publish a footprint of 0.5m "radius" at origin
  auto radius = 0.5;
  collision_tester->publishFootprint(radius, 0.0, 0.0, "test_base_frame", node->now());

  // Publish an empty costmap
  // It should not hit anything in an empty costmap
  collision_tester->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_TRUE(controller->isTrajectoryCollisionFree(dock_pose, true, false));

  // Set a dock in the costmap of 0.2x1.5m at 2m in front of the robot
  // It should hit the dock because the robot is 0.5m wide and the dock pose is at 1.75
  // But it does not hit because the collision tolerance is 0.3m
  collision_tester->setRectangle(0.2, 1.5, 2.0, -0.75, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_TRUE(controller->isTrajectoryCollisionFree(dock_pose, true, false));

  // Set an object between the robot and the dock
  // It should hit the object
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 0.2, 1.0, -0.1, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_FALSE(controller->isTrajectoryCollisionFree(dock_pose, true, false));

  // Set the collision tolerance to 0 to ensure all obstacles in the path are detected
  controller->setCollisionTolerance(0.0);

  // Set a dock in the costmap of 0.2x1.5m at 2m in front of the robot
  // Now it should hit the dock
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 1.5, 2.0, -0.75, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_FALSE(controller->isTrajectoryCollisionFree(dock_pose, true, false));

  collision_tester->deactivate();
}

TEST(ControllerTests, CollisionCheckerDockBackward) {
  auto collision_tester = std::make_shared<TestCollisionChecker>("collision_test");
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model

  nav2_util::declare_parameter_if_not_declared(
    node, "controller.footprint_topic", rclcpp::ParameterValue("test_footprint"));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.costmap_topic", rclcpp::ParameterValue("test_costmap_raw"));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.projection_time", rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.simulation_time_step", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.dock_collision_threshold", rclcpp::ParameterValue(0.3));

  auto controller = std::make_unique<opennav_docking::ControllerFixture>(
    node, tf, "test_base_frame", "test_base_frame");
  collision_tester->configure();
  collision_tester->activate();

  // Set the pose of the dock at 1.75m behind the robot
  auto dock_pose = collision_tester->setPose(-1.75, 0.0, 0.0);

  // Publish a footprint of 0.5m "radius" at origin
  auto radius = 0.5;
  collision_tester->publishFootprint(radius, 0.0, 0.0, "test_base_frame", node->now());

  // Publish an empty costmap
  // It should not hit anything in an empty costmap
  collision_tester->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_TRUE(controller->isTrajectoryCollisionFree(dock_pose, true, true));

  // Set a dock in the costmap of 0.2x1.5m at 2m behind the robot
  // It should hit the dock because the robot is 0.5m wide and the dock pose is at -1.75
  // But it does not hit because the collision tolerance is 0.3m
  collision_tester->setRectangle(0.2, 1.5, -2.1, -0.75, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_TRUE(controller->isTrajectoryCollisionFree(dock_pose, true, true));

  // Set an object between the robot and the dock
  // It should hit the object
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 0.2, -1.0, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_FALSE(controller->isTrajectoryCollisionFree(dock_pose, true, true));

  // Set the collision tolerance to 0 to ensure all obstacles in the path are detected
  controller->setCollisionTolerance(0.0);

  // Set a dock in the costmap of 0.2x1.5m at 2m behind the robot
  // Now it should hit the dock
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 1.5, -2.1, -0.75, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_FALSE(controller->isTrajectoryCollisionFree(dock_pose, true, true));

  collision_tester->deactivate();
}

TEST(ControllerTests, CollisionCheckerUndockBackward) {
  auto collision_tester = std::make_shared<TestCollisionChecker>("collision_test");
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model

  nav2_util::declare_parameter_if_not_declared(
    node, "controller.footprint_topic", rclcpp::ParameterValue("test_footprint"));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.costmap_topic", rclcpp::ParameterValue("test_costmap_raw"));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.projection_time", rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.simulation_time_step", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.dock_collision_threshold", rclcpp::ParameterValue(0.3));

  auto controller = std::make_unique<opennav_docking::ControllerFixture>(
    node, tf, "test_base_frame", "test_base_frame");
  collision_tester->configure();
  collision_tester->activate();

  // Set the staging pose at 1.75m behind the robot
  auto staging_pose = collision_tester->setPose(-1.75, 0.0, 0.0);

  // Publish a footprint of 0.5m "radius" at origin
  auto radius = 0.5;
  collision_tester->publishFootprint(radius, 0.0, 0.0, "test_base_frame", node->now());

  // Publish an empty costmap
  // It should not hit anything in an empty costmap
  collision_tester->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_TRUE(controller->isTrajectoryCollisionFree(staging_pose, false, true));

  // Set a dock in the costmap of 0.2x1.5m in front of the robot. The robot is docked
  // It should hit the dock because the robot is 0.5m wide and the robot pose is at 1.75
  // But it does not hit because the collision tolerance is 0.3m
  collision_tester->setRectangle(0.2, 1.5, 0.25, -0.75, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_TRUE(controller->isTrajectoryCollisionFree(staging_pose, false, true));

  // Set an object beyond the staging pose
  // It should hit the object
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 0.2, -1.75 - 0.5, -0.1, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_FALSE(controller->isTrajectoryCollisionFree(staging_pose, false, true));

  // Set an object between the robot and the staging pose
  // It should hit the object
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 0.2, -1.0, -0.1, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_FALSE(controller->isTrajectoryCollisionFree(staging_pose, false, true));

  // Set the collision tolerance to 0 to ensure all obstacles in the path are detected
  controller->setCollisionTolerance(0.0);

  // Set a dock in the costmap of 0.2x1.5m in front of the robot. The robot is docked
  // Now it should hit the dock
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 1.5, 0.25, -0.75, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_FALSE(controller->isTrajectoryCollisionFree(staging_pose, false, true));

  collision_tester->deactivate();
}

TEST(ControllerTests, CollisionCheckerUndockForward) {
  auto collision_tester = std::make_shared<TestCollisionChecker>("collision_test");
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model

  nav2_util::declare_parameter_if_not_declared(
    node, "controller.footprint_topic", rclcpp::ParameterValue("test_footprint"));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.costmap_topic", rclcpp::ParameterValue("test_costmap_raw"));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.projection_time", rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.simulation_time_step", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.dock_collision_threshold", rclcpp::ParameterValue(0.3));

  auto controller = std::make_unique<opennav_docking::ControllerFixture>(
    node, tf, "test_base_frame", "test_base_frame");
  collision_tester->configure();
  collision_tester->activate();

  // Set the staging pose at 1.75m in the front of the robot
  auto staging_pose = collision_tester->setPose(1.75, 0.0, 0.0);

  // Publish a footprint of 0.5m "radius"
  auto radius = 0.5;
  collision_tester->publishFootprint(radius, 0.0, 0.0, "test_base_frame", node->now());

  // Publish an empty costmap
  // It should not hit anything in an empty costmap
  collision_tester->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_TRUE(controller->isTrajectoryCollisionFree(staging_pose, false, false));

  // Set a dock in the costmap of 0.2x1.5m at 0.5m behind the robot. The robot is docked
  // It should not hit anything because the robot is docked and the trajectory is backward
  collision_tester->setRectangle(0.2, 1.5, -0.35, -0.75, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_TRUE(controller->isTrajectoryCollisionFree(staging_pose, false, false));

  // Set an object beyond the staging pose
  // It should hit the object
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 0.3, 1.75 + 0.5, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_FALSE(controller->isTrajectoryCollisionFree(staging_pose, false, false));

  // Set an object between the robot and the staging pose
  // It should hit the object
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 0.2, 1.0, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_FALSE(controller->isTrajectoryCollisionFree(staging_pose, false, false));

  // Set the collision tolerance to 0 to ensure all obstacles in the path are detected
  controller->setCollisionTolerance(0.0);

  // Set a dock in the costmap of 0.2x1.5m at 0.5m behind the robot. The robot is docked
  // Now it should hit the dock
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 1.5, -0.35, -0.75, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_FALSE(controller->isTrajectoryCollisionFree(staging_pose, false, false));

  collision_tester->deactivate();
}


}  // namespace opennav_docking
