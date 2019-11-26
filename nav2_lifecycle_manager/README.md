#### Background on lifecycle enabled nodes
The lifecycle enabled nodes in the navigation stack (e.g. map_server, planner_server, controller_server) have lifecycle functions namely on_configure(), on_activate(), on_deactivate(), on_cleanup(), on_shutdown(), and on_error(). Managed lifecycle nodes allow roslaunch to ensure that all components have been instantiated correctly before it allows any component to begin executing its behaviour. It will also allow nodes to be restarted or replaced on-line. More details about managed nodes can be found on [ROS2 Design website](https://design.ros2.org/articles/node_lifecycle.html).


#### nav2_lifecycle_manager
The lifecycle manager is used to change the states of it's managed nodes in order to achieve a controlled startup, shutdown, reset, pause or resume of the navigation stack. 

It presents a _"lifecycle_manager/manage_nodes"_ service, from which one can request the _startup_, _shutdown_, _reset_, _pause_, or _resume_ functions to be called. Based on this service request, the lifecycle manager calls the necessary lifecycle services in the lifecycle managed nodes. Currently, the RVIZ panel uses the _"lifecycle_manager/manage_nodes"_ sevice when user presses the buttons on the RVIZ panel (e.g.,startup, reset, shutdown, etc.).


The lifecycle manager has a default nodes list as shown below:
 ```
 {"map_server", "amcl", "planner_server", "controller_server", "recoveries_server", "bt_navigator"}
 ```

In order to start the navigation stack and be able to navigate, we need the necessary nodes to be on _configured_ and _activated_ states. Thus, for example when _startup_ is requested from the lifecycle manager's _manage_nodes_ service, it calls the _on_configure()_ and _on_activate()_ on the lifecycle enabled nodes in the node list.


<img src="./doc/diagram_lifecycle_manager.JPG" title="" width="100%" align="middle">

The UML diagram below shows the sequence of service calls once the _startup_ is requested from the lifecycle manager.

<img src="./doc/uml_lifecycle_manager.JPG" title="Lifecycle manager UML diagram" width="100%" align="middle">