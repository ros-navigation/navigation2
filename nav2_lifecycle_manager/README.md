### Background on lifecycle enabled nodes
The lifecycle enabled nodes in the navigation stack (e.g. map_server, planner_server, controller_server) have lifecycle functions namely ```on_configure()```, ```on_activate()```, ```on_deactivate()```, ```on_cleanup()```, ```on_shutdown()```, and ```on_error()```. Managed lifecycle nodes allow ros2launch to ensure that all required nodes have been instantiated correctly before it allows any nodes to begin executing its behaviour. It will also allow nodes to be restarted or replaced on-line. More details about managed nodes can be found on [ROS2 Design website](https://design.ros2.org/articles/node_lifecycle.html).


### nav2_lifecycle_manager
The lifecycle manager is used to change the states of it's managed nodes in order to achieve a controlled startup, shutdown, reset, pause or resume of the navigation stack. 

It presents a ```lifecycle_manager/manage_nodes``` service, from which one can request the _startup_, _shutdown_, _reset_, _pause_, or _resume_ functions to be called. Based on this service request, the lifecycle manager calls the necessary lifecycle services in the lifecycle managed nodes. Currently, the RVIZ panel uses this ```lifecycle_manager/manage_nodes``` service when user presses the buttons on the RVIZ panel (e.g.,startup, reset, shutdown, etc.).


The lifecycle manager has a default nodes list for all the nodes that it manages.

In order to start the navigation stack and be able to navigate, we need the necessary nodes to be on _configured_ and _activated_ states. Thus, for example when _startup_ is requested from the lifecycle manager's _manage_nodes_ service, it calls the _on_configure()_ and _on_activate()_ on the lifecycle enabled nodes in the node list.


The diagram below shows an _example_ of a list of managed nodes, and how it interfaces with the lifecycle manager.
<img src="./doc/diagram_lifecycle_manager.JPG" title="" width="100%" align="middle">

The UML diagram below shows the sequence of service calls once the _startup_ is requested from the lifecycle manager.

<img src="./doc/uml_lifecycle_manager.JPG" title="Lifecycle manager UML diagram" width="100%" align="middle">