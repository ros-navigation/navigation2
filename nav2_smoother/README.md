# Nav2 Smoother

The Nav2 Smoother is a [Task Server](../doc/requirements/requirements.md) that implements the `nav2_msgs::action::SmoothPath` action server.

A task server implementing the `nav2_msgs::action::SmoothPath` action server is responsible for optimizing global plan or its local section around the robot, given the computed path from the planner module in `nav2_planner`. The nav2_smoother package is designed to be loaded with plugins for path optimization. The plugins need to implement functions in the virtual base class defined in the `smoother` header file in `nav2_core` package.


Currently available smoother plugin is: [Ceres costaware smoother](https://github.com/ros-planning/navigation2/tree/main/nav2_ceres_costaware_smoother).