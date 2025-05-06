# nav2\_toolkit

A modular ROS 2 package offering quality-of-life (QOL) tools to improve the usability, and development experience when working with the navigation2 stack.

## Included Utilities

### 1. **Persistent Pose Saver**

A ROS 2 node for persisting and restoring the robot's last known pose.

**Features:**

* Save the current pose to disk periodically
* Write safely using an atomic temp-file swap
* Restore the robot pose automatically on command
* Start/Stop pose saving dynamically with services
* Parameter-configurable save interval and file path
* Support for saving to multiple file locations

### 2. **Goal Resume & Pause**

A utility to pause ongoing Nav2 goals and resume on command or restart.

**Features:**

* Pause the current goal without canceling it
* Resume paused goals seamlessly
* Integrate with Nav2's action interface
* Optional timeout or precondition checks before resuming

### 3. **BaseFootprintPublisher**

