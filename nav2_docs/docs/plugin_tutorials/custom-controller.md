# Creating a Custom Controller Plugin

This tutorial guides you through creating a custom controller (local planner) plugin for Nav2. Controllers are responsible for generating velocity commands to follow a global path while avoiding obstacles.

## Prerequisites

- ROS 2 Humble or later installed
- Nav2 stack installed
- Completed the [Custom Planner Tutorial](custom-planner.md) or equivalent understanding
- Understanding of robot kinematics and velocity control

## Overview

We will create a simple proportional controller that steers the robot toward the path using basic proportional control. This demonstrates the controller plugin interface while keeping the algorithm straightforward.

## Step 1: Create the Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake simple_controller \
  --dependencies rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util pluginlib geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
```

## Step 2: Create the Header File

Create `include/simple_controller/simple_controller.hpp`:

```cpp
#ifndef SIMPLE_CONTROLLER__SIMPLE_CONTROLLER_HPP_
#define SIMPLE_CONTROLLER__SIMPLE_CONTROLLER_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace simple_controller
{

class SimpleController : public nav2_core::Controller
{
public:
  SimpleController() = default;
  ~SimpleController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPath(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  void reset() override;

private:
  // Find the closest point on path to the robot
  size_t findClosestPoint(const geometry_msgs::msg::PoseStamped & pose);

  // Find lookahead point on path
  geometry_msgs::msg::PoseStamped getLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & pose,
    double lookahead_dist);

  // Transform pose to robot frame
  geometry_msgs::msg::PoseStamped transformToRobotFrame(
    const geometry_msgs::msg::PoseStamped & pose);

  // Check if pose is in collision
  bool isCollisionFree(const geometry_msgs::msg::PoseStamped & pose);

  // Node and infrastructure
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;

  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("SimpleController")};
  rclcpp::Clock::SharedPtr clock_;

  // Frames
  std::string global_frame_;
  std::string robot_frame_;

  // Path storage
  nav_msgs::msg::Path global_path_;
  std::mutex path_mutex_;

  // Control parameters
  double desired_linear_vel_;
  double max_linear_vel_;
  double max_angular_vel_;
  double lookahead_dist_;
  double transform_tolerance_;

  // Proportional gains
  double k_linear_;
  double k_angular_;

  // Speed limit
  double speed_limit_;
  bool speed_limit_is_percentage_;
};

}  // namespace simple_controller

#endif  // SIMPLE_CONTROLLER__SIMPLE_CONTROLLER_HPP_
```

## Step 3: Implement the Plugin

Create `src/simple_controller.cpp`:

```cpp
#include "simple_controller/simple_controller.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace simple_controller
{

void SimpleController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::ControllerException("Failed to lock parent node");
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  global_frame_ = costmap_ros_->getGlobalFrameID();
  robot_frame_ = costmap_ros_->getBaseFrameID();

  // Declare parameters
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_linear_vel", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".k_linear", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".k_angular", rclcpp::ParameterValue(2.0));

  // Get parameters
  node->get_parameter(name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(name_ + ".max_linear_vel", max_linear_vel_);
  node->get_parameter(name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(name_ + ".k_linear", k_linear_);
  node->get_parameter(name_ + ".k_angular", k_angular_);

  // Initialize speed limit
  speed_limit_ = max_linear_vel_;
  speed_limit_is_percentage_ = false;

  RCLCPP_INFO(logger_, "Configured controller: %s", name_.c_str());
}

void SimpleController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up controller: %s", name_.c_str());
  global_path_ = nav_msgs::msg::Path();
}

void SimpleController::activate()
{
  RCLCPP_INFO(logger_, "Activating controller: %s", name_.c_str());
}

void SimpleController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating controller: %s", name_.c_str());
}

void SimpleController::setPath(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  global_path_ = path;
  RCLCPP_DEBUG(logger_, "Received new path with %zu poses", path.poses.size());
}

geometry_msgs::msg::TwistStamped SimpleController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock(path_mutex_);

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = robot_frame_;

  // Check for empty path
  if (global_path_.poses.empty()) {
    throw nav2_core::ControllerException("Received empty path");
  }

  // Check if we've reached the goal
  geometry_msgs::msg::PoseStamped goal_pose = global_path_.poses.back();
  if (goal_checker != nullptr) {
    geometry_msgs::msg::Pose current_pose = pose.pose;
    geometry_msgs::msg::Pose goal = goal_pose.pose;

    // If goal checker says we're done, return zero velocity
    geometry_msgs::msg::Twist zero_vel;
    if (goal_checker->isGoalReached(current_pose, goal, zero_vel)) {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      return cmd_vel;
    }
  }

  // Get lookahead point
  geometry_msgs::msg::PoseStamped lookahead_point = getLookaheadPoint(pose, lookahead_dist_);

  // Transform lookahead point to robot frame
  geometry_msgs::msg::PoseStamped lookahead_in_robot = transformToRobotFrame(lookahead_point);

  // Calculate control commands
  double dx = lookahead_in_robot.pose.position.x;
  double dy = lookahead_in_robot.pose.position.y;
  double distance = std::hypot(dx, dy);

  // Angle to lookahead point
  double angle_to_lookahead = std::atan2(dy, dx);

  // Calculate velocities using proportional control
  double linear_vel = k_linear_ * distance;
  double angular_vel = k_angular_ * angle_to_lookahead;

  // Apply speed limit
  double effective_max_linear;
  if (speed_limit_is_percentage_) {
    effective_max_linear = max_linear_vel_ * (speed_limit_ / 100.0);
  } else {
    effective_max_linear = std::min(max_linear_vel_, speed_limit_);
  }

  // Slow down when turning sharply
  double curvature = std::abs(angular_vel) / std::max(linear_vel, 0.1);
  if (curvature > 1.0) {
    linear_vel *= (1.0 / curvature);
  }

  // Clamp velocities
  linear_vel = std::clamp(linear_vel, 0.0, effective_max_linear);
  angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);

  // Set output
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;

  return cmd_vel;
}

void SimpleController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;

  if (percentage) {
    RCLCPP_INFO(logger_, "Speed limit set to %.1f%%", speed_limit);
  } else {
    RCLCPP_INFO(logger_, "Speed limit set to %.2f m/s", speed_limit);
  }
}

void SimpleController::reset()
{
  RCLCPP_INFO(logger_, "Resetting controller: %s", name_.c_str());
  global_path_ = nav_msgs::msg::Path();
}

size_t SimpleController::findClosestPoint(const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_path_.poses.empty()) {
    return 0;
  }

  double min_dist = std::numeric_limits<double>::max();
  size_t closest_idx = 0;

  for (size_t i = 0; i < global_path_.poses.size(); ++i) {
    double dx = global_path_.poses[i].pose.position.x - pose.pose.position.x;
    double dy = global_path_.poses[i].pose.position.y - pose.pose.position.y;
    double dist = std::hypot(dx, dy);

    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  return closest_idx;
}

geometry_msgs::msg::PoseStamped SimpleController::getLookaheadPoint(
  const geometry_msgs::msg::PoseStamped & pose,
  double lookahead_dist)
{
  // Find closest point on path
  size_t closest_idx = findClosestPoint(pose);

  // Look ahead from closest point
  double accumulated_dist = 0.0;
  size_t lookahead_idx = closest_idx;

  for (size_t i = closest_idx; i < global_path_.poses.size() - 1; ++i) {
    double dx = global_path_.poses[i + 1].pose.position.x -
      global_path_.poses[i].pose.position.x;
    double dy = global_path_.poses[i + 1].pose.position.y -
      global_path_.poses[i].pose.position.y;
    accumulated_dist += std::hypot(dx, dy);

    if (accumulated_dist >= lookahead_dist) {
      lookahead_idx = i + 1;
      break;
    }
    lookahead_idx = i + 1;
  }

  // Return the lookahead point (or last point if path is shorter than lookahead)
  return global_path_.poses[lookahead_idx];
}

geometry_msgs::msg::PoseStamped SimpleController::transformToRobotFrame(
  const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped robot_pose;

  try {
    robot_pose = tf_->transform(
      pose, robot_frame_,
      rclcpp::Duration::from_seconds(transform_tolerance_));
  } catch (tf2::TransformException & ex) {
    throw nav2_core::ControllerException(
      "Could not transform pose to robot frame: " + std::string(ex.what()));
  }

  return robot_pose;
}

bool SimpleController::isCollisionFree(const geometry_msgs::msg::PoseStamped & pose)
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
    return false;
  }

  unsigned char cost = costmap_->getCost(mx, my);
  return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

}  // namespace simple_controller

// Register the plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(simple_controller::SimpleController, nav2_core::Controller)
```

## Step 4: Create the Plugin XML

Create `simple_controller_plugin.xml`:

```xml
<library path="simple_controller">
  <class type="simple_controller::SimpleController" base_class_type="nav2_core::Controller">
    <description>
      A simple proportional controller that follows paths using lookahead-based steering.
    </description>
  </class>
</library>
```

## Step 5: Update CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.5)
project(simple_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Create library
add_library(${PROJECT_NAME} SHARED
  src/simple_controller.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES simple_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin
pluginlib_export_plugin_description_file(nav2_core simple_controller_plugin.xml)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
```

## Step 6: Update package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>simple_controller</name>
  <version>1.0.0</version>
  <description>A simple proportional controller plugin for Nav2</description>

  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>nav2_core</depend>
  <depend>nav2_costmap_2d</depend>
  <depend>nav2_util</depend>
  <depend>pluginlib</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## Step 7: Configure Nav2

Add to your Nav2 parameters file:

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "simple_controller::SimpleController"
      desired_linear_vel: 0.5
      max_linear_vel: 1.0
      max_angular_vel: 1.0
      lookahead_dist: 0.6
      transform_tolerance: 0.1
      k_linear: 1.0
      k_angular: 2.0
```

## Step 8: Build and Test

```bash
cd ~/ros2_ws
colcon build --packages-select simple_controller
source install/setup.bash
```

## Key Implementation Details

### Path Following Strategy

The controller uses a lookahead-based approach:

1. Find the closest point on the path to the robot
2. Look ahead a fixed distance along the path
3. Calculate the angle to the lookahead point
4. Apply proportional control for steering

### Thread Safety

The path is protected with a mutex since `setPath()` and `computeVelocityCommands()` may be called from different threads.

### Speed Limiting

The `setSpeedLimit()` method supports both absolute (m/s) and percentage-based limits, allowing integration with speed zones or external constraints.

## Enhancements

### Add Collision Checking

```cpp
geometry_msgs::msg::TwistStamped SimpleController::computeVelocityCommands(...)
{
  // ... existing code ...

  // Check if lookahead point is collision-free
  if (!isCollisionFree(lookahead_point)) {
    // Slow down or stop
    linear_vel *= 0.5;
    RCLCPP_WARN(logger_, "Lookahead point in potential collision zone");
  }

  // ... rest of method ...
}
```

### Add Path Pruning

```cpp
void SimpleController::prunePath(const geometry_msgs::msg::PoseStamped & pose)
{
  size_t closest_idx = findClosestPoint(pose);
  if (closest_idx > 0) {
    global_path_.poses.erase(
      global_path_.poses.begin(),
      global_path_.poses.begin() + closest_idx);
  }
}
```

### Add Velocity Smoothing

```cpp
double SimpleController::smoothVelocity(double target, double current, double max_accel, double dt)
{
  double diff = target - current;
  double max_change = max_accel * dt;
  return current + std::clamp(diff, -max_change, max_change);
}
```

## Troubleshooting

### Robot oscillates

- Reduce `k_angular` gain
- Increase `lookahead_dist`

### Robot moves too slowly

- Increase `desired_linear_vel`
- Increase `k_linear` gain

### Transform errors

- Check that TF tree is complete
- Increase `transform_tolerance`
- Verify frame IDs are correct

## Next Steps

- Implement pure pursuit or Stanley controller
- Add dynamic reconfigure for tuning
- Implement trajectory rollout for collision avoidance
- Add support for Ackermann kinematics
