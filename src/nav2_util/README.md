# Nav2 Util

The `nav2_util` package contains utilities abstracted from individual packages which may find use in other uses. Some examples of things you'll find here:

- Geometry utilities for computing distances and values in paths
- A Nav2 specific lifecycle node wrapper for boilerplate code and useful common utilities like `declare_parameter_if_not_declared()`
- Simplified service clients
- Simplified action servers
- Transformation and robot pose helpers
- Twist Subscriber and Twist Publisher

The long-term aim is for these utilities to find more permanent homes in other packages (within and outside of Nav2) or migrate to the raw tools made available in ROS 2.

## Twist Publisher and Twist Subscriber for commanded velocities

### Background 

The Twist Publisher and Twist Subscriber are utility classes to assist NAV2 transition from 
[Twist](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Twist.msg) to [TwistStamped](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/TwistStamped.msg).

Details on the migration are found in [#1594](https://github.com/ros-planning/navigation2/issues/1594).

Certain applications of NAV2, such as in ROS Aerial mandate the usage of `TwistStamped`, while many other applications still use `Twist`.

The utility has the following effect:
* Allows use of either `Twist` or `TwistStamped`, controlled by ROS parameter `enable_stamped_cmd_vel`
* Preserves existing topic names without duplication of data

Every node in `nav2` that subscribes or publishes velocity commands with `Twist` now supports this optional behavior.
The behavior up through ROS 2 Iron is preserved - using `Twist`. In a future ROS 2 version, when enough of the
ROS ecosystem has moved to `TwistStamped`, the default may change. 
