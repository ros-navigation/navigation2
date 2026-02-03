# Creating a Custom Global Planner Plugin

This tutorial walks through creating a custom global planner plugin for Nav2. By the end, you will have a fully functional planner that can be loaded and used by the Nav2 planner server.

## Prerequisites

- ROS 2 Humble or later installed
- Nav2 stack installed (`sudo apt install ros-humble-navigation2`)
- Basic understanding of C++ and ROS 2 concepts
- Familiarity with CMake and colcon build system

## Overview

We will create a simple straight-line planner that generates a direct path from start to goal. While not practical for real navigation (it ignores obstacles), this example demonstrates the complete plugin development workflow.

## Step 1: Create the Package

Create a new ROS 2 package for your planner:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake straight_line_planner \
  --dependencies rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util pluginlib geometry_msgs nav_msgs tf2_ros
```

## Step 2: Create the Header File

Create `include/straight_line_planner/straight_line_planner.hpp`:

```cpp
#ifndef STRAIGHT_LINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define STRAIGHT_LINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace straight_line_planner
{

class StraightLinePlanner : public nav2_core::GlobalPlanner
{
public:
  StraightLinePlanner() = default;
  ~StraightLinePlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  // Interpolation resolution in meters
  double interpolation_resolution_;

  // Node, TF buffer, and costmap
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;

  // Plugin name and logger
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("StraightLinePlanner")};
  rclcpp::Clock::SharedPtr clock_;

  // Global frame
  std::string global_frame_;
};

}  // namespace straight_line_planner

#endif  // STRAIGHT_LINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
```

## Step 3: Implement the Plugin

Create `src/straight_line_planner.cpp`:

```cpp
#include "straight_line_planner/straight_line_planner.hpp"

#include <cmath>
#include <string>
#include <memory>

#include "nav2_util/node_utils.hpp"
#include "nav2_core/planner_exceptions.hpp"

namespace straight_line_planner
{

void StraightLinePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("Failed to lock parent node");
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Declare and get parameters
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));

  node->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

  RCLCPP_INFO(
    logger_, "Configured plugin %s with interpolation_resolution: %.2f",
    name_.c_str(), interpolation_resolution_);
}

void StraightLinePlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up plugin %s", name_.c_str());
  costmap_ = nullptr;
}

void StraightLinePlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s", name_.c_str());
}

void StraightLinePlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s", name_.c_str());
}

nav_msgs::msg::Path StraightLinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;

  // Check if costmap is available
  if (!costmap_) {
    throw nav2_core::PlannerException("Costmap is not initialized");
  }

  // Validate frames match
  if (start.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException(
      "Start pose frame " + start.header.frame_id +
      " does not match global frame " + global_frame_);
  }

  if (goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException(
      "Goal pose frame " + goal.header.frame_id +
      " does not match global frame " + global_frame_);
  }

  // Set path header
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;

  // Calculate distance and number of interpolation points
  double dx = goal.pose.position.x - start.pose.position.x;
  double dy = goal.pose.position.y - start.pose.position.y;
  double distance = std::hypot(dx, dy);

  int num_points = static_cast<int>(distance / interpolation_resolution_);
  if (num_points < 2) {
    num_points = 2;
  }

  // Generate interpolated path
  for (int i = 0; i <= num_points; ++i) {
    // Check for cancellation
    if (cancel_checker()) {
      RCLCPP_INFO(logger_, "Planning cancelled");
      return nav_msgs::msg::Path();
    }

    double t = static_cast<double>(i) / static_cast<double>(num_points);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = start.pose.position.x + t * dx;
    pose.pose.position.y = start.pose.position.y + t * dy;
    pose.pose.position.z = 0.0;

    // Interpolate orientation using slerp (simplified: use goal orientation for all points)
    if (i == num_points) {
      pose.pose.orientation = goal.pose.orientation;
    } else {
      // Calculate heading towards goal
      double yaw = std::atan2(dy, dx);
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
    }

    path.poses.push_back(pose);
  }

  RCLCPP_INFO(
    logger_, "Created path with %zu poses from (%.2f, %.2f) to (%.2f, %.2f)",
    path.poses.size(),
    start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  return path;
}

}  // namespace straight_line_planner

// Register the plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(straight_line_planner::StraightLinePlanner, nav2_core::GlobalPlanner)
```

## Step 4: Create the Plugin XML

Create `straight_line_planner_plugin.xml`:

```xml
<library path="straight_line_planner">
  <class type="straight_line_planner::StraightLinePlanner" base_class_type="nav2_core::GlobalPlanner">
    <description>
      A simple straight-line global planner that creates direct paths between start and goal poses.
    </description>
  </class>
</library>
```

## Step 5: Update CMakeLists.txt

Replace the contents of `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.5)
project(straight_line_planner)

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
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)

# Set dependencies
set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  geometry_msgs
  nav_msgs
  tf2_ros
)

# Create library
add_library(${PROJECT_NAME} SHARED
  src/straight_line_planner.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install plugin description file
install(FILES straight_line_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin
pluginlib_export_plugin_description_file(nav2_core straight_line_planner_plugin.xml)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
```

## Step 6: Update package.xml

Ensure your `package.xml` contains:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>straight_line_planner</name>
  <version>1.0.0</version>
  <description>A simple straight-line global planner plugin for Nav2</description>

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
  <depend>tf2_ros</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## Step 7: Build the Plugin

```bash
cd ~/ros2_ws
colcon build --packages-select straight_line_planner
source install/setup.bash
```

## Step 8: Configure Nav2 to Use Your Plugin

Create or modify your Nav2 parameters file to use the new planner:

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "straight_line_planner::StraightLinePlanner"
      interpolation_resolution: 0.1
```

## Step 9: Test the Plugin

1. Launch your robot simulation or hardware
2. Launch Nav2 with your custom configuration
3. Send a navigation goal using RViz or the command line:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

## Adding Obstacle Checking

To make the planner practical, add collision checking:

```cpp
bool StraightLinePlanner::isPathCollisionFree(const nav_msgs::msg::Path & path)
{
  for (const auto & pose : path.poses) {
    unsigned int mx, my;
    if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
      return false;  // Point is outside costmap
    }

    unsigned char cost = costmap_->getCost(mx, my);
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;  // Point is in collision
    }
  }
  return true;
}
```

## Best Practices

1. **Always check cancel_checker()** periodically during long computations
2. **Validate input frames** match the expected global frame
3. **Use RCLCPP_INFO/WARN/ERROR** for appropriate logging levels
4. **Throw nav2_core::PlannerException** for unrecoverable errors
5. **Return empty paths** for recoverable failures
6. **Properly implement lifecycle methods** to manage resources
7. **Document parameters** in your plugin description

## Troubleshooting

### Plugin not found

Check that:
- The plugin XML is installed correctly
- `pluginlib_export_plugin_description_file` is called in CMakeLists.txt
- The library name matches in the XML and CMakeLists.txt

### Segmentation fault

Common causes:
- Accessing costmap_ before it's initialized
- Using invalid node weak_ptr
- Missing null checks on shared pointers

### Planning fails immediately

Check:
- Frame IDs match between start, goal, and global frame
- Costmap is properly initialized
- Parameters are declared and retrieved correctly

## Next Steps

- Add A* or Dijkstra search for obstacle-aware planning
- Implement path smoothing
- Add visualization markers for debugging
- Create unit tests for your planner
