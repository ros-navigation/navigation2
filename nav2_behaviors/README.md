# Behaviors

The `nav2_behaviors` package implements a task server for executing behaviors.

The package defines:
- A `TimedBehavior` template which is used as a base class to implement specific timed behavior action server - but not required.
- The  `Backup`, `DriveOnHeading`, `Spin` and `Wait` behaviors.

The only required class a behavior must derive from is the `nav2_core/behavior.hpp` class, which implements the pluginlib interface the behavior server will use to dynamically load your behavior. The `nav2_behaviors/timed_behavior.hpp` derives from this class and implements a generic action server for a timed behavior behavior (e.g. calls an implmentation function on a regular time interval to compute a value) but **this is not required** if it is not helpful. A behavior does not even need to be an action if you do not wish, it may be a service or other interface. However, most motion and behavior primitives are probably long-running and make sense to be modeled as actions, so the provided `timed_behavior.hpp` helps in managing the complexity to simplify new behavior development, described more below.

The value of the centralized behavior server is to **share resources** amongst several behaviors that would otherwise be independent nodes. Subscriptions to TF, costmaps, and more can be quite heavy and add non-trivial compute costs to a robot system. By combining these independent behaviors into a single server, they may share these resources while retaining complete independence in execution and interface.

See its [Configuration Guide Page](https://docs.nav2.org/configuration/packages/configuring-behavior-server.html) for additional parameter descriptions and a [tutorial about writing behaviors](https://docs.nav2.org/plugin_tutorials/docs/writing_new_behavior_plugin.html).

See the [Navigation Plugin list](https://docs.nav2.org/plugins/index.html) for a list of the currently known and available planner plugins.

The `TimedBehavior` template makes use of a [nav2_util::TwistPublisher](../nav2_util/README.md#twist-publisher-and-twist-subscriber-for-commanded-velocities).
The `AssistedTeleop` behavior makes use of a [nav2_util::TwistSubscriber](../nav2_util/README.md#twist-publisher-and-twist-subscriber-for-commanded-velocities).