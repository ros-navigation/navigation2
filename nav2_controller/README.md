# Nav2 Controller

The Nav2 Controller is an [Execution Module](../doc/requirements/requirements.md) that implements the `nav2_msgs::action::FollowPath` action server.

An execution module implementing the `nav2_msgs::action::FollowPath` action server is responsible for generating command velocities for the robot, given the computed path from the planner module in `nav2_planner`. The nav2_controller package is designed to be loaded with plugins for path execution. The plugins need to implement functions in the virtual base class defined in the `controller` header file in `nav2_core` package.


Currently available controller plugins are: DWB, and [TEB (dashing release)](https://github.com/rst-tu-dortmund/teb_local_planner/tree/dashing-devel).