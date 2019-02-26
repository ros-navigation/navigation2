# Lifecycle

The Lifecycle project provides a foundation for implementing lifecycle nodes in the Navigation 2 system.

## Overview

The motivation for integrating [ROS2 managed nodes](http://design.ros2.org/articles/node_lifecycle.html) into the Navigation 2 system is to 

* Ensure that the nav2 system starts up and shuts down reliably
* Clearly separate the non-deterministic code from the deterministic code (running in the Active state), setting us up for a subsequent project enable the nav2 software for deterministic (real-time) performance.
* Simplify the model for node configuration

### Reliable start-up and shutdown

Using lifecycle nodes, the system can launched in a controlled fashion, verifying each step of the process. Lifecycle nodes can be controlled from a launch script or from a separate controller node (the current implementation uses a controller node).

### Separation of deterministic and non-deterministic code

The `Active` state of the lifecycle node is intended to be where the node's execution is deterministic. The other start-up state transitions allow a node to configure itself and allocate its resources in preparation for running in the active state.

### Node configuration

Lifecycle nodes have an explicit `Configuring` state transition where the node can read its configuration parameters and configure itself. Reconfiguring a node can consist of taking the node back to the `Unconfigured` state, updating the node parameters, and transitioning the node through the intermediate states back to the `Active` state. ROS 1-style dynamic parameters, where a node is reconfigured on-the-fly while it is running, should be reserved for simple parameter updates that don't result in non-deterministic behavior, such as allocating buffers.

## Implementation

This project provides two base classes, `LifecycleNode` and `ILifeCycleNode`. The first is a wrapper for the `rclcpp_lifecycle::LifecycleNode` class. This provides an opportunity to provide functionality currently missing in the `rclcpp_lifecycle::LifecycleNode` class or to provide our own functionality for each (lifecycle) node in the system.

The `ILifecycleNode` abstract base class is used by those classes in the system that aren't themselves nodes, but provide functionality to nodes such that they require a node SharedPtr to do things like creating publishers and subscribers. To be in sync with their parent objects, these support objects also need to walk through the same callbacks as used in the lifecycle nodes. That is, when the parent object is configuring itself in the `OnConfigure` callback, it can dispatch OnConfigure for its child objects as well.

### rclcpp::Node support

Because some functionality in ROS2 is not yet Lifecycle enabled, the `nav2_lifecycle::LifecycleNode` class provides an option to allow for creating a non-lifecycle ROS2 node that the derived class can use to interface to the non-lifecycle-enabled classes, such as `MessageFilter` and `TransformListener`. If the derived class specifies that the base class should create one of these nodes, the `nav2_lifecycle::LifecycleNode` base class automatically launches a thread to service this node's messages.

### Lifecycle service client

This project also implements a `LifecycleServiceClient` class that can be used to interface to a lifecycle-enabled node. This class is used by the nav2 controller node to transition the lifecycle nodes through their various states.