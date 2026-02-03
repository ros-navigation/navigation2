# Nav2 Plugin API Reference

This document provides comprehensive API reference for all Nav2 plugin interfaces, including parameters, methods, and configuration examples.

## Table of Contents

1. [GlobalPlanner API](#globalplanner-api)
2. [Controller API](#controller-api)
3. [Behavior API](#behavior-api)
4. [Costmap Layer API](#costmap-layer-api)
5. [Smoother API](#smoother-api)
6. [Common Utilities](#common-utilities)

---

## GlobalPlanner API

### Base Class

```cpp
namespace nav2_core {
  class GlobalPlanner;
}
```

### Required Headers

```cpp
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
```

### Method Reference

#### configure()

```cpp
virtual void configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `parent` | `LifecycleNode::WeakPtr` | Parent lifecycle node for parameter access |
| `name` | `std::string` | Plugin instance name for parameter namespacing |
| `tf` | `shared_ptr<tf2_ros::Buffer>` | Transform buffer for coordinate lookups |
| `costmap_ros` | `shared_ptr<Costmap2DROS>` | Costmap wrapper for map access |

#### createPlan()

```cpp
virtual nav_msgs::msg::Path createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker) = 0;
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `start` | `PoseStamped` | Starting pose in global frame |
| `goal` | `PoseStamped` | Goal pose in global frame |
| `cancel_checker` | `function<bool()>` | Returns true if planning should be cancelled |

**Returns:** `nav_msgs::msg::Path` - Planned path or empty path on failure

**Exceptions:** `nav2_core::PlannerException` on critical failures

### Configuration Example

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
      use_final_approach_orientation: false
```

### Common Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tolerance` | `double` | 0.5 | Goal tolerance in meters |
| `use_astar` | `bool` | false | Use A* instead of Dijkstra |
| `allow_unknown` | `bool` | true | Plan through unknown space |

---

## Controller API

### Base Class

```cpp
namespace nav2_core {
  class Controller;
}
```

### Required Headers

```cpp
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
```

### Method Reference

#### setPath()

```cpp
virtual void setPath(const nav_msgs::msg::Path & path) = 0;
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `path` | `nav_msgs::msg::Path` | Global path to follow |

#### computeVelocityCommands()

```cpp
virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker) = 0;
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `pose` | `PoseStamped` | Current robot pose |
| `velocity` | `Twist` | Current robot velocity |
| `goal_checker` | `GoalChecker*` | Goal checker instance |

**Returns:** `TwistStamped` - Velocity command

**Exceptions:** `nav2_core::ControllerException` on failures

#### setSpeedLimit()

```cpp
virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `speed_limit` | `double` | Maximum speed value |
| `percentage` | `bool` | If true, limit is percentage (0-100) |

### Configuration Example

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: true

    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.5
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.8
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.6
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.25
      use_rotate_to_heading: true
      allow_reversing: false
      rotate_to_heading_min_angle: 0.785
      max_angular_accel: 3.2
      max_robot_pose_search_dist: 10.0
```

### Common Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `desired_linear_vel` | `double` | 0.5 | Desired forward velocity (m/s) |
| `max_linear_vel` | `double` | 1.0 | Maximum linear velocity (m/s) |
| `max_angular_vel` | `double` | 1.0 | Maximum angular velocity (rad/s) |
| `lookahead_dist` | `double` | 0.6 | Lookahead distance (m) |
| `transform_tolerance` | `double` | 0.1 | TF lookup timeout (s) |

---

## Behavior API

### Base Class

```cpp
namespace nav2_core {
  class Behavior;
}
```

### CostmapInfoType Enum

```cpp
enum class CostmapInfoType {
  NONE = 0,   // No costmap needed
  LOCAL = 1,  // Local costmap only
  GLOBAL = 2, // Global costmap only
  BOTH = 3    // Both costmaps needed
};
```

### Required Headers

```cpp
#include "nav2_core/behavior.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
```

### Method Reference

#### configure()

```cpp
virtual void configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> local_collision_checker,
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> global_collision_checker) = 0;
```

#### getResourceInfo()

```cpp
virtual CostmapInfoType getResourceInfo() = 0;
```

**Returns:** `CostmapInfoType` indicating required costmap resources

### TimedBehavior Methods

For behaviors extending `nav2_behaviors::TimedBehavior<ActionT>`:

#### onRun()

```cpp
virtual ResultStatus onRun(
  const std::shared_ptr<const ActionT::Goal> command) = 0;
```

Called once when behavior is triggered.

#### onCycleUpdate()

```cpp
virtual ResultStatus onCycleUpdate() = 0;
```

Called repeatedly until behavior completes.

### Configuration Example

```yaml
behavior_server:
  ros__parameters:
    local_costmap_topic: local_costmap/costmap_raw
    global_costmap_topic: global_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    global_footprint_topic: global_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]

    spin:
      plugin: "nav2_behaviors::Spin"

    backup:
      plugin: "nav2_behaviors::BackUp"

    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"

    wait:
      plugin: "nav2_behaviors::Wait"
```

---

## Costmap Layer API

### Base Class

```cpp
namespace nav2_costmap_2d {
  class Layer;
}
```

### Required Headers

```cpp
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
```

### Cost Values

```cpp
namespace nav2_costmap_2d {
  static const unsigned char NO_INFORMATION = 255;
  static const unsigned char LETHAL_OBSTACLE = 254;
  static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
  static const unsigned char MAX_NON_OBSTACLE = 252;
  static const unsigned char FREE_SPACE = 0;
}
```

### Method Reference

#### updateBounds()

```cpp
virtual void updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x, double * max_y) = 0;
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `robot_x` | `double` | Robot X position (world coords) |
| `robot_y` | `double` | Robot Y position (world coords) |
| `robot_yaw` | `double` | Robot orientation (radians) |
| `min_x/y` | `double*` | Minimum bounds (in/out) |
| `max_x/y` | `double*` | Maximum bounds (in/out) |

#### updateCosts()

```cpp
virtual void updateCosts(
  Costmap2D & master_grid,
  int min_i, int min_j,
  int max_i, int max_j) = 0;
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `master_grid` | `Costmap2D&` | Reference to master costmap |
| `min_i/j` | `int` | Minimum cell indices |
| `max_i/j` | `int` | Maximum cell indices |

### Configuration Example

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      footprint_padding: 0.03
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

---

## Smoother API

### Base Class

```cpp
namespace nav2_core {
  class Smoother;
}
```

### Required Headers

```cpp
#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
```

### Method Reference

#### configure()

```cpp
virtual void configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) = 0;
```

#### smooth()

```cpp
virtual bool smooth(
  nav_msgs::msg::Path & path,
  const rclcpp::Duration & max_time) = 0;
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `path` | `Path&` | Path to smooth (modified in place) |
| `max_time` | `Duration` | Maximum allowed smoothing time |

**Returns:** `true` if completed, `false` if interrupted by time limit

### Configuration Example

```yaml
smoother_server:
  ros__parameters:
    smoother_plugins: ["simple_smoother"]

    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true
```

---

## Common Utilities

### Parameter Declaration

```cpp
#include "nav2_util/node_utils.hpp"

// Declare parameter if not already declared
nav2_util::declare_parameter_if_not_declared(
  node, name + ".param_name", rclcpp::ParameterValue(default_value));

// Get parameter value
double param;
node->get_parameter(name + ".param_name", param);
```

### Exception Classes

```cpp
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_core/controller_exceptions.hpp"

// Planner exceptions
throw nav2_core::PlannerException("Error message");
throw nav2_core::StartOccupied("Start position in collision");
throw nav2_core::GoalOccupied("Goal position in collision");
throw nav2_core::NoValidPathCouldBeFound("No path found");

// Controller exceptions
throw nav2_core::ControllerException("Error message");
throw nav2_core::ControllerTFError("Transform error");
throw nav2_core::NoValidControl("Cannot compute valid control");
```

### Logging

```cpp
RCLCPP_DEBUG(logger_, "Debug message: %s", value);
RCLCPP_INFO(logger_, "Info message");
RCLCPP_WARN(logger_, "Warning message");
RCLCPP_ERROR(logger_, "Error message");
RCLCPP_FATAL(logger_, "Fatal error");
```

### Transform Utilities

```cpp
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Lookup transform
geometry_msgs::msg::TransformStamped transform;
try {
  transform = tf_->lookupTransform(
    target_frame, source_frame,
    tf2::TimePointZero,
    rclcpp::Duration::from_seconds(0.1));
} catch (tf2::TransformException & ex) {
  RCLCPP_ERROR(logger_, "Transform error: %s", ex.what());
}

// Transform pose
geometry_msgs::msg::PoseStamped transformed_pose;
tf2::doTransform(original_pose, transformed_pose, transform);
```

### Costmap Access

```cpp
// Get costmap pointer
nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();

// World to map conversion
unsigned int mx, my;
if (costmap->worldToMap(wx, wy, mx, my)) {
  unsigned char cost = costmap->getCost(mx, my);
}

// Map to world conversion
double wx, wy;
costmap->mapToWorld(mx, my, wx, wy);

// Get costmap properties
double resolution = costmap->getResolution();
unsigned int size_x = costmap->getSizeInCellsX();
unsigned int size_y = costmap->getSizeInCellsY();
double origin_x = costmap->getOriginX();
double origin_y = costmap->getOriginY();
```

### Geometry Utilities

```cpp
#include "nav2_util/geometry_utils.hpp"
#include <cmath>

// Calculate distance
double distance = std::hypot(dx, dy);

// Calculate heading
double yaw = std::atan2(dy, dx);

// Quaternion to yaw
double yaw = tf2::getYaw(pose.orientation);

// Yaw to quaternion
geometry_msgs::msg::Quaternion q;
q.x = 0.0;
q.y = 0.0;
q.z = std::sin(yaw / 2.0);
q.w = std::cos(yaw / 2.0);
```
