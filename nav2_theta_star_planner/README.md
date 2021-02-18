#Nav2 Theta Star Planner

The Theta Star Planner is a global planning plugin meant to be used with the Nav2 Planner Server.
The `nav2_theta_star_planner` implements a highly optimized version of the Theta Star Planner (specifically the [Lazy Theta\* P variant](http://idm-lab.org/bib/abstracts/papers/aaai10b.pdf)) meant 
to plan any-angle paths using A\*. Further improvements to the path quality were made by taking into account the costs from a 2D costmap. 
The planner supports differential-drive and omni-directional robots.

##Features
- Allows to tune the path quality from being taut to being smooth (this depends on the resolution of the map)
- Uses the potential field from the costmap to penalise high cost regions
- Is well suited for smaller robots of the omni-directional and differential drive kind
- As it considers the costmap traversal cost during execution it tends to smoothen the paths automatically, thus mitigating the need to smoothen the path. (The presence of sharp turns depends on the resolution of the map, and it decreases as the map resolution increases.)


<!-- (TODO(Anshu-man567) : add images and gifs) -->
 
##Parameters 

The parameters of the planner are :
- ``` .how_many_corners ``` : to choose between 4-connected and 8-connected graph expnasions
- ``` .w_euc_cost ``` : weight applied to make the paths taut
- ``` .w_traversal_cost ``` : weight applied to steer the path away from collisions and cost
- ``` .w_heuristic_cost ``` : weight applied to push the node expansion towards the goal

Below are the default values of the parameters :
```
planner_server:
  ros__parameters:
    planner_plugin_types: ["nav2_theta_star_planner/ThetaStarPlanner"]
    use_sim_time: True
    planner_plugin_ids: ["GridBased"]
    GridBased:
      how_many_corners: 8
      w_euc_cost: 4.0
      w_traversal_cost: 6.25
      w_heuristic_cost: 1.0
```

<!-- TODO (Anshu-man567) : write usage notes / help to decide when to use -->




 
