# Nav2 Plugin Interfaces

This document provides comprehensive documentation for all Nav2 plugin base classes, including method signatures, virtual methods that must be overridden, and lifecycle hooks.

## Table of Contents

1. [GlobalPlanner Interface](#globalplanner-interface)
2. [Controller Interface](#controller-interface)
3. [Behavior Interface](#behavior-interface)
4. [Costmap Layer Interface](#costmap-layer-interface)
5. [Smoother Interface](#smoother-interface)

---

## GlobalPlanner Interface

**Header:** `nav2_core/global_planner.hpp`

**Namespace:** `nav2_core`

The `GlobalPlanner` class defines the interface for all global path planning algorithms in Nav2.

### Class Definition

```cpp
namespace nav2_core
{

class GlobalPlanner
{
public:
  using Ptr = std::shared_ptr<GlobalPlanner>;

  virtual ~GlobalPlanner() {}

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  virtual void cleanup() = 0;

  virtual void activate() = 0;

  virtual void deactivate() = 0;

  virtual nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) = 0;
};

}  // namespace nav2_core
```

### Methods to Override

| Method | Purpose | Required |
|--------|---------|----------|
| `configure()` | Initialize plugin with node, TF buffer, and costmap | Yes |
| `cleanup()` | Release resources on shutdown | Yes |
| `activate()` | Enable plugin and start threads | Yes |
| `deactivate()` | Disable plugin and stop threads | Yes |
| `createPlan()` | Generate path from start to goal | Yes |

### Method Details

#### configure()

```cpp
virtual void configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;
```

**Parameters:**
- `parent` - Weak pointer to the parent lifecycle node
- `name` - Plugin instance name for namespacing parameters
- `tf` - Transform buffer for coordinate frame lookups
- `costmap_ros` - Costmap wrapper providing map access

**Responsibilities:**
- Declare and retrieve plugin parameters
- Store references to tf buffer and costmap
- Initialize internal data structures
- Create any required publishers/subscribers

#### createPlan()

```cpp
virtual nav_msgs::msg::Path createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker) = 0;
```

**Parameters:**
- `start` - Starting pose in the global frame
- `goal` - Goal pose in the global frame
- `cancel_checker` - Function returning true if planning should be cancelled

**Returns:** `nav_msgs::msg::Path` containing the planned path

**Responsibilities:**
- Validate start and goal poses
- Check cancel_checker periodically during long computations
- Return empty path on failure
- Throw `nav2_core::PlannerException` on critical errors

---

## Controller Interface

**Header:** `nav2_core/controller.hpp`

**Namespace:** `nav2_core`

The `Controller` class defines the interface for local trajectory following algorithms.

### Class Definition

```cpp
namespace nav2_core
{

class Controller
{
public:
  using Ptr = std::shared_ptr<Controller>;

  virtual ~Controller() {}

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  virtual void cleanup() = 0;

  virtual void activate() = 0;

  virtual void deactivate() = 0;

  virtual void setPath(const nav_msgs::msg::Path & path) = 0;

  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) = 0;

  virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;

  virtual void reset() {}

  virtual bool cancel() { return true; }
};

}  // namespace nav2_core
```

### Methods to Override

| Method | Purpose | Required |
|--------|---------|----------|
| `configure()` | Initialize plugin with node, TF buffer, and costmap | Yes |
| `cleanup()` | Release resources on shutdown | Yes |
| `activate()` | Enable plugin and start threads | Yes |
| `deactivate()` | Disable plugin and stop threads | Yes |
| `setPath()` | Receive new global path to follow | Yes |
| `computeVelocityCommands()` | Calculate velocity command | Yes |
| `setSpeedLimit()` | Apply speed constraints | Yes |
| `reset()` | Reset controller state | No (optional) |
| `cancel()` | Cancel current control action | No (optional) |

### Method Details

#### setPath()

```cpp
virtual void setPath(const nav_msgs::msg::Path & path) = 0;
```

**Parameters:**
- `path` - Global path to follow

**Responsibilities:**
- Store the path for trajectory following
- Perform any necessary path transformations
- Reset internal state if needed

#### computeVelocityCommands()

```cpp
virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker) = 0;
```

**Parameters:**
- `pose` - Current robot pose
- `velocity` - Current robot velocity
- `goal_checker` - Goal checker for determining arrival

**Returns:** `geometry_msgs::msg::TwistStamped` with velocity command

**Responsibilities:**
- Transform path to robot frame if needed
- Calculate appropriate velocities
- Check for collisions
- Throw `nav2_core::ControllerException` on failure

#### setSpeedLimit()

```cpp
virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;
```

**Parameters:**
- `speed_limit` - Maximum speed value
- `percentage` - If true, speed_limit is a percentage (0-100); if false, absolute value in m/s

---

## Behavior Interface

**Header:** `nav2_core/behavior.hpp`

**Namespace:** `nav2_core`

The `Behavior` class defines the interface for recovery and auxiliary behaviors.

### Class Definition

```cpp
namespace nav2_core
{

enum class CostmapInfoType
{
  NONE = 0,
  LOCAL = 1,
  GLOBAL = 2,
  BOTH = 3
};

class Behavior
{
public:
  using Ptr = std::shared_ptr<Behavior>;

  virtual ~Behavior() {}

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> local_collision_checker,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> global_collision_checker) = 0;

  virtual void cleanup() = 0;

  virtual void activate() = 0;

  virtual void deactivate() = 0;

  virtual CostmapInfoType getResourceInfo() = 0;
};

}  // namespace nav2_core
```

### Methods to Override

| Method | Purpose | Required |
|--------|---------|----------|
| `configure()` | Initialize plugin with collision checkers | Yes |
| `cleanup()` | Release resources on shutdown | Yes |
| `activate()` | Enable plugin and start threads | Yes |
| `deactivate()` | Disable plugin and stop threads | Yes |
| `getResourceInfo()` | Declare costmap requirements | Yes |

### Method Details

#### configure()

```cpp
virtual void configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> local_collision_checker,
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> global_collision_checker) = 0;
```

**Parameters:**
- `parent` - Weak pointer to the parent lifecycle node
- `name` - Plugin instance name
- `tf` - Transform buffer
- `local_collision_checker` - Collision checker using local costmap
- `global_collision_checker` - Collision checker using global costmap

#### getResourceInfo()

```cpp
virtual CostmapInfoType getResourceInfo() = 0;
```

**Returns:** Enum indicating which costmaps the behavior requires

**Values:**
- `CostmapInfoType::NONE` - No costmap needed
- `CostmapInfoType::LOCAL` - Local costmap only
- `CostmapInfoType::GLOBAL` - Global costmap only
- `CostmapInfoType::BOTH` - Both costmaps needed

---

## Costmap Layer Interface

**Header:** `nav2_costmap_2d/layer.hpp`

**Namespace:** `nav2_costmap_2d`

The `Layer` class defines the interface for costmap layer plugins.

### Class Definition

```cpp
namespace nav2_costmap_2d
{

class Layer
{
public:
  Layer();
  virtual ~Layer() {}

  void initialize(
    LayeredCostmap * parent,
    std::string name,
    tf2_ros::Buffer * tf,
    const nav2_util::LifecycleNode::WeakPtr & node,
    rclcpp::CallbackGroup::SharedPtr callback_group);

  virtual void activate() {}
  virtual void deactivate() {}
  virtual void reset() = 0;

  virtual bool isClearable() { return false; }

  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) = 0;

  virtual void updateCosts(
    Costmap2D & master_grid,
    int min_i, int min_j,
    int max_i, int max_j) = 0;

  virtual void matchSize() {}

  virtual void onFootprintChanged() {}

  std::string getName() const { return name_; }

  bool isCurrent() const { return current_; }
  bool isEnabled() const { return enabled_; }
  void setCurrent(bool current) { current_ = current; }

protected:
  virtual void onInitialize() {}

  LayeredCostmap * layered_costmap_;
  std::string name_;
  tf2_ros::Buffer * tf_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  bool current_;
  bool enabled_;
};

}  // namespace nav2_costmap_2d
```

### Methods to Override

| Method | Purpose | Required |
|--------|---------|----------|
| `onInitialize()` | Custom initialization logic | No (recommended) |
| `activate()` | Enable layer processing | No |
| `deactivate()` | Disable layer processing | No |
| `reset()` | Clear layer data | Yes |
| `isClearable()` | Indicate if layer supports clearing | No |
| `updateBounds()` | Calculate update region | Yes |
| `updateCosts()` | Apply costs to master costmap | Yes |
| `matchSize()` | Resize to match parent costmap | No |
| `onFootprintChanged()` | Handle footprint updates | No |

### Method Details

#### updateBounds()

```cpp
virtual void updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x, double * max_y) = 0;
```

**Parameters:**
- `robot_x`, `robot_y`, `robot_yaw` - Robot pose in map frame
- `min_x`, `min_y`, `max_x`, `max_y` - Bounding box to expand

**Responsibilities:**
- Expand the bounding box to include areas that need updating
- Only expand, never shrink the bounds
- Consider sensor range and new observations

#### updateCosts()

```cpp
virtual void updateCosts(
  Costmap2D & master_grid,
  int min_i, int min_j,
  int max_i, int max_j) = 0;
```

**Parameters:**
- `master_grid` - Reference to the master costmap
- `min_i`, `min_j`, `max_i`, `max_j` - Cell indices of the update region

**Responsibilities:**
- Apply cost values to cells within the specified region
- Use appropriate cost combination methods (max, overwrite, etc.)

---

## Smoother Interface

**Header:** `nav2_core/smoother.hpp`

**Namespace:** `nav2_core`

The `Smoother` class defines the interface for path smoothing algorithms.

### Class Definition

```cpp
namespace nav2_core
{

class Smoother
{
public:
  using Ptr = std::shared_ptr<Smoother>;

  virtual ~Smoother() {}

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) = 0;

  virtual void cleanup() = 0;

  virtual void activate() = 0;

  virtual void deactivate() = 0;

  virtual bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time) = 0;
};

}  // namespace nav2_core
```

### Methods to Override

| Method | Purpose | Required |
|--------|---------|----------|
| `configure()` | Initialize plugin with costmap and footprint | Yes |
| `cleanup()` | Release resources on shutdown | Yes |
| `activate()` | Enable plugin | Yes |
| `deactivate()` | Disable plugin | Yes |
| `smooth()` | Smooth the given path | Yes |

### Method Details

#### configure()

```cpp
virtual void configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) = 0;
```

**Parameters:**
- `parent` - Weak pointer to the parent lifecycle node
- `name` - Plugin instance name
- `tf` - Transform buffer
- `costmap_sub` - Subscriber providing costmap updates
- `footprint_sub` - Subscriber providing footprint updates

#### smooth()

```cpp
virtual bool smooth(
  nav_msgs::msg::Path & path,
  const rclcpp::Duration & max_time) = 0;
```

**Parameters:**
- `path` - Path to smooth (modified in place)
- `max_time` - Maximum time allowed for smoothing

**Returns:** `true` if smoothing completed, `false` if interrupted by time limit

**Responsibilities:**
- Modify path in place to improve smoothness
- Respect the time limit
- Maintain collision-free property if possible
- Preserve start and end poses

---

## Common Patterns

### Parameter Declaration

```cpp
void MyPlugin::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, ...)
{
  node_ = parent;
  auto node = parent.lock();
  logger_ = node->get_logger();
  name_ = name;

  // Declare parameters with defaults
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".my_param", rclcpp::ParameterValue(0.5));

  // Retrieve parameter values
  node->get_parameter(name_ + ".my_param", my_param_);
}
```

### Exception Handling

```cpp
nav_msgs::msg::Path MyPlanner::createPlan(...)
{
  if (!costmap_ros_->getCostmap()) {
    throw nav2_core::PlannerException("Costmap not available");
  }

  // Planning logic...

  if (path.poses.empty()) {
    throw nav2_core::PlannerException("Failed to find valid path");
  }

  return path;
}
```

### Cancellation Checking

```cpp
nav_msgs::msg::Path MyPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  while (/* planning loop */) {
    if (cancel_checker()) {
      RCLCPP_INFO(logger_, "Planning cancelled");
      return nav_msgs::msg::Path();
    }
    // Continue planning...
  }
}
```
