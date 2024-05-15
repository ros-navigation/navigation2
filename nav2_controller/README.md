# Nav2 Controller

The Nav2 Controller is a Task Server in Nav2 that implements the `nav2_msgs::action::FollowPath` action server.

An execution module implementing the `nav2_msgs::action::FollowPath` action server is responsible for generating command velocities for the robot, given the computed path from the planner module in `nav2_planner`. The nav2_controller package is designed to be loaded with multiple plugins for path execution. The plugins need to implement functions in the virtual base class defined in the `controller` header file in `nav2_core` package. It also contains progress checkers and goal checker plugins to abstract out that logic from specific controller implementations.

See the [Navigation Plugin list](https://docs.nav2.org/plugins/index.html) for a list of the currently known and available controller plugins. 

See its [Configuration Guide Page](https://docs.nav2.org/configuration/packages/configuring-controller-server.html) for additional parameter descriptions and a [tutorial about writing controller plugins](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html).

The `ControllerServer` makes use of a [nav2_util::TwistPublisher](../nav2_util/README.md#twist-publisher-and-twist-subscriber-for-commanded-velocities).
