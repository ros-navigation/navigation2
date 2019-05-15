# Nav2 World Model

`nav2_world_model` is a package containing an exposed environmental representation. Today, it uses the `nav2_costmap_2d` layered costmap as the world model. In the future, this is the entry point for applications to request or view information about the environment around it. The implementations such as costmaps or height maps will provide the buffering of data and representations as they require. They are then wrapped for generalized use for path planning and control.

## ROS1 Comparison

ROS1 Navigation contains `costmap_2d` which is directly used as the evironmental model. This package is able to consume `nav2_costmap_2d` as an implementation of a world model, but can also utilize other world models to suit the needs of a specific application. Rather than querying `costmap_2d` for information, applications will query `nav2_world_model` which will in turn use the current world model and retrieve the information requested.

## Future

* Additional implementations/support for different environmental representations like `grid_maps`, height maps, traversibility maps, etc. 
* Utilities for converting types for use in other applications like control and planning. 
