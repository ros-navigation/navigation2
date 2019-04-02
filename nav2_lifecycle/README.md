# Lifecycle

The Lifecycle project provides temporary functionality for implementing lifecycle nodes in the Navigation 2 system. This directory will be removed once lifecycle-related issues are fixed in various ROS2 dependencies.

## Overview

The motivation for integrating [ROS2 managed nodes](http://design.ros2.org/articles/node_lifecycle.html) into the Navigation 2 system is to 

* Ensure that the nav2 system starts up and shuts down reliably
* Clearly separate the non-deterministic code from the deterministic code (running in the Active state), setting us up for a subsequent project enable the nav2 software for deterministic (real-time) performance.
* Simplify the model for node configuration

### Reliable start-up and shutdown

Using lifecycle nodes, the system can be launched in a controlled fashion, verifying each step of the process. Lifecycle nodes can be controlled from a launch script or from a separate controller node (the current implementation uses a controller node).

### Separation of deterministic and non-deterministic code

During the `Active` state of the lifecycle node, the node's execution is intended to be deterministic. The other start-up state transitions allow a node to configure itself and allocate its resources in preparation for running in the active state.

### Node configuration

Lifecycle nodes have an explicit `Configuring` state transition where the node can read its configuration parameters and configure itself. Reconfiguring a node can consist of taking the node back to the `Unconfigured` (or `Inactive`) state, updating the node parameters, and transitioning the node through the intermediate states back to the `Active` state. ROS 1-style dynamic parameters, where a node is reconfigured on-the-fly while it is running, should be reserved for simple parameter updates that don't result in non-deterministic behavior, such as allocating buffers.

## Implementation

This project provides two classes, `LifecycleNode` and `LifecycleHelperInterface`. 

The first, `nav2_lifecycle::LifecycleNode` class is a wrapper for ROS's `rclcpp_lifecycle::LifecycleNode`. This class provides an option, **use_rclcpp_node**, to create a non-lifecycle ROS2 node:

```cpp
class LifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifecycleNode(
    const std::string & node_name,
    const std::string & namespace_ = "",
    bool use_rclcpp_node = false,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~LifecycleNode();

protected:
  // Whether or not to create a local rclcpp::Node which can be used for ROS2 classes that don't
  // yet support lifecycle nodes
  bool use_rclcpp_node_;

  // The local node
  rclcpp::Node::SharedPtr rclcpp_node_;

  // When creating a local node, this class will launch a separate thread created to spin the node
  std::unique_ptr<std::thread> rclcpp_thread_;
};
```

If **use_rclcpp_node** is set to true, the class will create a non-lifecycle node, setting the rclcpp_node_ member variable. It will also automatically create a thread for this local node which will process messages for the thread, as shown in the LifecycleNode constructor:


```cpp
LifecycleNode::LifecycleNode(
  const std::string & node_name,
  const std::string & namespace_,
  bool use_rclcpp_node,
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, namespace_, options),
  use_rclcpp_node_(use_rclcpp_node)
{
  if (use_rclcpp_node_) {
    rclcpp_node_ = std::make_shared<rclcpp::Node>(node_name + "_rclcpp_node", namespace_);
    rclcpp_thread_ = std::make_unique<std::thread>(
      [](rclcpp::Node::SharedPtr node) {rclcpp::spin(node);}, rclcpp_node_
    );
  }
}
```
Any lifecycle node classes derived from nav2_lifecycle::LifecycleNode and set use_rclcpp_node in the constructor will have this rclcpp_node_ available to use to interface to any ROS2 classes that don't yet support lifecycle nodes. For example, the following code from the lifecycle version of the AMCL node initializes a tf2_ros::buffer, tf2_ros::TransformListener, and tf2_ros::TransformBroadcaster using the rclcpp_node_:

```cpp
void
AmclNode::initTransforms()
{
  RCLCPP_INFO(get_logger(), "initTransforms");

  // Initialize transform listener and broadcaster
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rclcpp_node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, rclcpp_node_, false);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(rclcpp_node_);

   ...
}
```

The second class, `LifecycleHelperInterface` is an abstract base class that is used by those classes in the nav2 system that aren't themselves nodes, but require a node SharedPtr to create publishers and subscribers. To be in sync with their parent objects, these helper classes also need to walk through the same callbacks as used in the lifecycle nodes. That is, when the parent object is configuring itself in the `OnConfigure` callback, it can dispatch OnConfigure for its child objects as well. For example, the following code shows how a parent object might respond to an on_activate callback by calling helper objects that support the `LifecycleHelperInterface`:

```cpp
CallbackReturn
Parent::on_activate(const rclcpp_lifecycle::State & state)
{
  // Parent does its own activation

  // Delegates the on_activate message to is contained classes
  // Helpers could throw an exception upon failure
  helper1__->on_activate(state);
  helper2__->on_activate(state);

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}
```
