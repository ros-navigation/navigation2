# Creating a Custom Behavior Plugin

This tutorial guides you through creating a custom behavior (recovery) plugin for Nav2. Behaviors are actions executed when the robot encounters navigation failures, such as getting stuck or failing to find a valid path.

## Prerequisites

- ROS 2 Humble or later installed
- Nav2 stack installed
- Understanding of ROS 2 actions
- Familiarity with the Nav2 architecture

## Overview

We will create a "wiggle" behavior that oscillates the robot left and right to potentially free it from minor stuck situations. This demonstrates the behavior plugin interface and action server integration.

## Step 1: Create the Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake wiggle_behavior \
  --dependencies rclcpp rclcpp_action rclcpp_lifecycle nav2_core nav2_behaviors nav2_costmap_2d nav2_util pluginlib geometry_msgs nav2_msgs tf2_ros
```

## Step 2: Create the Header File

Create `include/wiggle_behavior/wiggle_behavior.hpp`:

```cpp
#ifndef WIGGLE_BEHAVIOR__WIGGLE_BEHAVIOR_HPP_
#define WIGGLE_BEHAVIOR__WIGGLE_BEHAVIOR_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace wiggle_behavior
{

using WiggleAction = nav2_msgs::action::BackUp;  // Reuse BackUp action structure

class WiggleBehavior : public nav2_behaviors::TimedBehavior<WiggleAction>
{
public:
  WiggleBehavior();
  ~WiggleBehavior() override = default;

  nav2_behaviors::ResultStatus onRun(
    const std::shared_ptr<const WiggleAction::Goal> command) override;

  nav2_behaviors::ResultStatus onCycleUpdate() override;

  void onConfigure() override;
  void onCleanup() override;

protected:
  // Behavior parameters
  double angular_vel_;
  double wiggle_duration_;
  int num_wiggles_;

  // Runtime state
  int current_wiggle_;
  bool wiggling_left_;
  rclcpp::Time wiggle_start_time_;
  std::chrono::milliseconds wiggle_period_;
};

}  // namespace wiggle_behavior

#endif  // WIGGLE_BEHAVIOR__WIGGLE_BEHAVIOR_HPP_
```

## Step 3: Implement the Plugin

Create `src/wiggle_behavior.cpp`:

```cpp
#include "wiggle_behavior/wiggle_behavior.hpp"

#include "nav2_util/node_utils.hpp"

namespace wiggle_behavior
{

WiggleBehavior::WiggleBehavior()
: TimedBehavior<WiggleAction>(),
  angular_vel_(0.5),
  wiggle_duration_(0.5),
  num_wiggles_(3),
  current_wiggle_(0),
  wiggling_left_(true)
{
}

void WiggleBehavior::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in WiggleBehavior::onConfigure");
  }

  // Declare parameters
  nav2_util::declare_parameter_if_not_declared(
    node, "angular_vel", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, "wiggle_duration", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, "num_wiggles", rclcpp::ParameterValue(3));

  // Get parameters
  node->get_parameter("angular_vel", angular_vel_);
  node->get_parameter("wiggle_duration", wiggle_duration_);
  node->get_parameter("num_wiggles", num_wiggles_);

  wiggle_period_ = std::chrono::milliseconds(
    static_cast<int>(wiggle_duration_ * 1000.0));

  RCLCPP_INFO(
    logger_,
    "WiggleBehavior configured: angular_vel=%.2f, duration=%.2f, wiggles=%d",
    angular_vel_, wiggle_duration_, num_wiggles_);
}

void WiggleBehavior::onCleanup()
{
  RCLCPP_INFO(logger_, "WiggleBehavior cleaning up");
}

nav2_behaviors::ResultStatus WiggleBehavior::onRun(
  const std::shared_ptr<const WiggleAction::Goal> /*command*/)
{
  RCLCPP_INFO(logger_, "Starting wiggle behavior");

  // Initialize state
  current_wiggle_ = 0;
  wiggling_left_ = true;
  wiggle_start_time_ = clock_->now();

  return nav2_behaviors::ResultStatus{nav2_behaviors::Status::SUCCEEDED, 0};
}

nav2_behaviors::ResultStatus WiggleBehavior::onCycleUpdate()
{
  auto current_time = clock_->now();
  auto elapsed = current_time - wiggle_start_time_;

  // Check if current wiggle is complete
  if (elapsed >= rclcpp::Duration(wiggle_period_)) {
    // Switch direction
    wiggling_left_ = !wiggling_left_;
    wiggle_start_time_ = current_time;

    if (!wiggling_left_) {
      // Completed one full wiggle (left + right)
      current_wiggle_++;
      RCLCPP_DEBUG(logger_, "Completed wiggle %d of %d", current_wiggle_, num_wiggles_);
    }
  }

  // Check if behavior is complete
  if (current_wiggle_ >= num_wiggles_) {
    // Stop the robot
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.angular.z = 0.0;
    vel_pub_->publish(cmd_vel);

    RCLCPP_INFO(logger_, "Wiggle behavior completed successfully");
    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::SUCCEEDED, 0};
  }

  // Check for collision before commanding velocity
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = 0.0;
  pose2d.y = 0.0;
  pose2d.theta = wiggling_left_ ? angular_vel_ * 0.1 : -angular_vel_ * 0.1;

  if (!local_collision_checker_->isCollisionFree(pose2d)) {
    RCLCPP_WARN(logger_, "Collision detected, aborting wiggle");
    geometry_msgs::msg::Twist stop_cmd;
    vel_pub_->publish(stop_cmd);
    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED, 0};
  }

  // Publish velocity command
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = wiggling_left_ ? angular_vel_ : -angular_vel_;
  vel_pub_->publish(cmd_vel);

  return nav2_behaviors::ResultStatus{nav2_behaviors::Status::RUNNING, 0};
}

}  // namespace wiggle_behavior

// Register plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(wiggle_behavior::WiggleBehavior, nav2_core::Behavior)
```

## Step 4: Create the Plugin XML

Create `wiggle_behavior_plugin.xml`:

```xml
<library path="wiggle_behavior">
  <class type="wiggle_behavior::WiggleBehavior" base_class_type="nav2_core::Behavior">
    <description>
      A recovery behavior that oscillates the robot left and right to help escape minor stuck situations.
    </description>
  </class>
</library>
```

## Step 5: Update CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.5)
project(wiggle_behavior)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  nav2_core
  nav2_behaviors
  nav2_costmap_2d
  nav2_util
  nav2_msgs
  pluginlib
  geometry_msgs
  tf2_ros
)

add_library(${PROJECT_NAME} SHARED
  src/wiggle_behavior.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES wiggle_behavior_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_core wiggle_behavior_plugin.xml)

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
  <name>wiggle_behavior</name>
  <version>1.0.0</version>
  <description>A wiggle recovery behavior plugin for Nav2</description>

  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>nav2_core</depend>
  <depend>nav2_behaviors</depend>
  <depend>nav2_costmap_2d</depend>
  <depend>nav2_util</depend>
  <depend>nav2_msgs</depend>
  <depend>pluginlib</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2_ros</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## Step 7: Configure Nav2

Add to your Nav2 parameters:

```yaml
behavior_server:
  ros__parameters:
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait", "wiggle"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
    wait:
      plugin: "nav2_behaviors::Wait"
    wiggle:
      plugin: "wiggle_behavior::WiggleBehavior"
      angular_vel: 0.5
      wiggle_duration: 0.5
      num_wiggles: 3
```

## Understanding TimedBehavior

The `nav2_behaviors::TimedBehavior` base class provides:

### Lifecycle Methods

- **onConfigure()**: Called during lifecycle configuration
- **onCleanup()**: Called during lifecycle cleanup
- **onActivate()**: Called during lifecycle activation (optional override)
- **onDeactivate()**: Called during lifecycle deactivation (optional override)

### Execution Methods

- **onRun()**: Called once when the behavior action is requested
- **onCycleUpdate()**: Called repeatedly at the configured frequency until completed

### Available Resources

- `node_`: Weak pointer to parent lifecycle node
- `logger_`: ROS logger for the behavior
- `clock_`: ROS clock for timing
- `vel_pub_`: Publisher for velocity commands
- `local_collision_checker_`: Collision checker using local costmap
- `global_collision_checker_`: Collision checker using global costmap
- `tf_`: Transform buffer

### Return Status

`ResultStatus` contains:
- `status`: One of `SUCCEEDED`, `FAILED`, or `RUNNING`
- `error_code`: Numeric error code (0 for success)

## Creating Custom Action Types

For behaviors that need different parameters, create a custom action:

### Define the Action (in a msgs package)

```
# WiggleAction.action
---
builtin_interfaces/Duration total_elapsed_time
---
builtin_interfaces/Duration elapsed_time
int32 current_wiggle
```

### Use the Custom Action

```cpp
#include "my_msgs/action/wiggle.hpp"

using WiggleAction = my_msgs::action::Wiggle;

class WiggleBehavior : public nav2_behaviors::TimedBehavior<WiggleAction>
{
  // ... implementation using custom action fields
};
```

## Collision Checking

Use the collision checkers to ensure safety:

```cpp
nav2_behaviors::ResultStatus WiggleBehavior::onCycleUpdate()
{
  // Create projected pose
  geometry_msgs::msg::Pose2D projected_pose;
  projected_pose.x = delta_x;
  projected_pose.y = delta_y;
  projected_pose.theta = delta_theta;

  // Check local costmap for collision
  if (!local_collision_checker_->isCollisionFree(projected_pose)) {
    RCLCPP_WARN(logger_, "Collision predicted, stopping");
    stopRobot();
    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED, 1};
  }

  // Continue behavior...
}
```

## Best Practices

### 1. Always Check for Collisions

Before executing any motion, verify it's collision-free:

```cpp
if (!local_collision_checker_->isCollisionFree(projected_pose)) {
  return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED, 0};
}
```

### 2. Respect Timeouts

Check for action preemption and timeouts:

```cpp
nav2_behaviors::ResultStatus onCycleUpdate()
{
  // TimedBehavior handles action preemption automatically
  // but you can add custom timeout logic if needed

  auto elapsed = clock_->now() - start_time_;
  if (elapsed > max_duration_) {
    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED, 2};
  }
}
```

### 3. Stop the Robot on Failure

Always command zero velocity when aborting:

```cpp
void stopRobot()
{
  geometry_msgs::msg::Twist stop_cmd;
  stop_cmd.linear.x = 0.0;
  stop_cmd.angular.z = 0.0;
  vel_pub_->publish(stop_cmd);
}
```

### 4. Provide Feedback

Update action feedback for monitoring:

```cpp
auto feedback = std::make_shared<WiggleAction::Feedback>();
feedback->elapsed_time = elapsed;
feedback->current_wiggle = current_wiggle_;
action_server_->publish_feedback(feedback);
```

## Testing the Behavior

### Manual Testing

Call the behavior action directly:

```bash
ros2 action send_goal /wiggle nav2_msgs/action/BackUp "{}"
```

### Integration with Behavior Trees

Add to your behavior tree:

```xml
<Recovery number_of_retries="3" name="RecoveryNode">
  <NavigateRecovery/>
  <SequenceStar name="RecoveryActions">
    <ClearEntireCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>
    <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
    <Action name="Wiggle" action_name="wiggle" />
    <Spin spin_dist="1.57"/>
    <Wait wait_duration="5"/>
    <BackUp backup_dist="0.15" backup_speed="0.025"/>
  </SequenceStar>
</Recovery>
```

## Troubleshooting

### Behavior not loading

- Verify plugin XML path is correct
- Check library name matches in CMakeLists.txt
- Ensure `pluginlib_export_plugin_description_file` is called

### Robot doesn't move

- Check velocity publisher is connected
- Verify collision checker isn't blocking motion
- Check parameter values are reasonable

### Behavior runs indefinitely

- Ensure exit conditions are properly checked
- Verify `SUCCEEDED` or `FAILED` is returned when complete

## Next Steps

- Create behaviors for specific scenarios (unstuck from doorways, etc.)
- Add costmap clearing during recovery
- Implement adaptive behavior based on environment
- Add visualization markers for debugging
