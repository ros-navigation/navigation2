# Nav2 Theta Star Planner
The Theta Star Planner is a global planning plugin meant to be used with the Nav2 Planner Server. The `nav2_theta_star_planner` implements a highly optimized version of the Theta Star Planner (specifically the [Lazy Theta\* P variant](http://idm-lab.org/bib/abstracts/papers/aaai10b.pdf)) meant to plan any-angle paths using A\*. The planner supports differential-drive and omni-directional robots.

## Features 
- Uses the costs from the costmap to penalise high cost regions
- Is well suited for smaller robots of the omni-directional and differential drive kind
- As it considers the costmap traversal cost during execution it tends to smoothen the paths automatically, thus mitigating the need to smoothen the path. (The presence of sharp turns depends on the resolution of the map, and it decreases as the map resolution increases.)
- Allows to control the path behavior to either be any angle directed or to be in the middle of the spaces
- The planner uses A\* search along with line of sight checks to form any-angle paths thus avoiding zig-zag paths that may be present in the usual implementation of A\*  

## Metrics
For the below example the planner took ~44ms to compute the path of 87.5m - 
![alt text](/img/00-37.png)

The parameters were set to - `w_euc_cost: 1.0`, `w_traversal_cost: 5.0`, `w_heuristic_cost: 1.0` and the `global_costmap`'s `inflation_layer` parameters are set as - `cost_scaling_factor:5.0`, `inflation_radius: 5.5`

## Cost Function Details
### Symbols and their meanings
**g(a)** - cost function cost for the node 'a'

**h(a)** - heuristic function cost for the node 'a'

**f(a)** - total cost (g(a) + h(a)) for the node 'a'

**LETHAL_COST** - a value of the costmap traversal cost that inscribes an obstacle with
respect to a function, value = 253

**curr** - represents the node whose neighbours are being added to the list

**neigh** - one of the neighboring nodes of curr

**par** - parent node of curr

**euc_cost(a,b)** - euclidean distance between the node type 'a' and type 'b'

**costmap_cost(a,b)** - the costmap traversal cost (ranges from 0 - 252, 254 = unknown value) between the node type 'a' and type 'b'

### Cost function
```
g(neigh) = g(curr) + w_euc_cost*euc_cost(curr, neigh) + w_traversal_cost*(costmap_cost(curr,neigh)/LETHAL_COST)^2
h(neigh) = w_heuristic_cost * euc_cost(neigh, par)
f(neigh) = g(neigh) + h(neigh)
```
Because of how the program works when the 'neigh' node is to be expanded, depending
on the result of the LOS check, (if the LOS check returns true) the value of g(neigh) might change to `g(par) +
w1*euc_cost(par, neigh) + w2*(costmap(par,neigh)/LETHAL_COST)^2`

## Parameters
The parameters of the planner are :
- ` .how_many_corners ` : to choose between 4-connected and 8-connected graph expansions, the accepted values are 4 and 8
- ` .w_euc_cost ` : weight applied on the length of the path. 
- ` .w_traversal_cost ` : it tunes how harshly the nodes of high cost are penalised. From the above g(neigh) equation you can see that the cost-aware component of the cost function forms a parabolic curve, thus this parameter would, on increasing its value, make that curve steeper allowing for a greater differentiation (as the delta of costs would increase, when the graph becomes steep) among the nodes of different costs.
- ` .w_heuristic_cost ` : it has been provided to have a admissible heuristic, so the recommendation would be to change its value only when required. Usually set is at the same value as the `w_euc_cost` or 1.0 (whichever is lower), though you may increase the value of `w_heuristic_cost` to speed up the process.

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
      w_traversal_cost: 7.0
      w_heuristic_cost: 1.0
```
Do note that the `global_costmap`'s `inflation_layer` values recommended for the above values of the parameter are - `cost_scaling_factor:1.0`, `inflation_radius: 5.5`

## Usage Notes

### Tuning the Parameters
Before starting off, do note that the costmap_cost(curr,neigh) component after being operated (before being multiplied to its parameter and being substituted in g(node)) varies from 0 to 1. Keep this in mind while tuning.

This planner uses the costs associated with each cell from the `global_costmap` as a measure of the point's proximity to the obstacles. Providing a gentle potential field that covers the entirety of the region (with only small pocket like regions of cost = 0) is recommended in order to achieve paths that pass through the middle of the spaces. A good starting point could be to set the `inflation_layer`'s parameters as - `cost_scaling_factor:10.0`, `inflation_radius: 5.5` and then to decrease the value of `cost_scaling_factor` to achieve the said potential field.

Providing a gentle potential field over the region allows the planner to compensate for the increase in path lengths which would allow for an increase in the distance from the obstacles, which around a corner allows for naturally smoothing the turns and thus removing the requirement for the use of an external path smoother.

In order to achieve paths that stay in the middle of the spaces set `w_traversal_cost` at a higher value than `w_euc_cost`. To begin with, you can set the parameters to its default values and then increase the value of `w_traversal_cost`, while also decreasing `w_euc_cost` to allow for an increase in the path length thus letting the path to settle in the middle regions of the spaces with lower costs on the costmap. While tuning the planner's parameters you can also change the `inflation_layer`'s parameters to tune the behavior of the paths.

### Path Smoothing
Because of how the cost function works, the output path has a natural tendency to form smooth curves around corners, though the smoothness of the path depends on how wide the turn is and the number of cells in that turns.

This planner is recommended to be used with local planners like DWB or TEB (or other any planner / controllers that form a local trajectory to be traversed) as these take into account the abrupt turns which might arise due to the planner not being able to find a smoother turns owing to the aforementioned reasons.

While smoother paths can be achieved by increasing the costmap resolution (ie using a costmap of 1cm resolution rather than a 5cm one) it is not recommended to do so as it comes at the cost of increased query times from the planner. Test the planners performance on the finer costmaps before making a switch to those costmaps. 
