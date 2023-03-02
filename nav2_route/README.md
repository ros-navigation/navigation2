# Nav2 Route Server

The Route Server is a Nav2 Task server to compliment the Planner Server's free-space planning capabilities with pre-defined Navigation Route Graph planning, created by [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/) while at [Samsung Research America](https://www.sra.samsung.com/) with assistence from [Josh Wallace](https://www.linkedin.com/in/joshua-wallace-b4848a113/) at Locus Robotics.
This graph has few rules associated with it and may be generated manually or automatically via AI, geometric, or probablistic techniques.
This package then takes a planning request and uses this graph to find a valid route through the environment via an optimal search-based algorithm.
It may also live monitor and analyze the route's process to execute custom behaviors on entering or leaving edges or achieving particular graph nodes.

There are plugin interfaces throughout the server to enable a great deal of application-specific customization:
- Custom search-prioritization behavior with edge scoring plugins (e.g. minimize distance or time, mark blocked routes, enact static or dynamic penalties for danger and application-specific knowledge, etc)
- Custom operations to perform during route execution, triggered when entering or leaving an edge or achieving a graph node on the path (e.g. open door, pause at node to wait for clearance, adjust maximum speed, etc)
- Parsers of navigation graph files to use any type of format desirable (e.g. geoJSON, OpenStreetMap)

Additionally, the parsers store **additional arbitrary metadata** specified within the navigation graph files to store information such as speed limits, added costs, or operations to perform.
Thus, we do not restrict the data that can be embedded in the navigation route graph for an application and this metadata is communicated to the edge scoring and operations plugins to adjust behavior.
Note however that plugins may also use outside information from topics, services, and actions for dynamic behavior or centralized knowledge sharing as well. 

## Features

- Optimized Dikjstra's planning algorithm modeled off of the Smac Planner A* implementation
- Cleverly designed for no run-time lookups on the graph during search (2 lookups to find the start and goal edges on request initialization)
- Use of Kd-trees for finding the nearest node to arbitrary start and goal poses in the graph
- Highly efficient graph representation to maximize caching in a single data structure containing both edges' and nodes' objects and relationships with localized information
- All edges are directional
- Data in files may be with respect to any frame in the TF tree and are transformed to a centralized frame automatically
- Action interface response returns both a sparse route of nodes and edges for client applications with navigation graph knowledge and `nav_msgs/Path` dense paths minimicking freespace planning for drop-in behavior replacement of the Planner Server.  
- Action interface request can process requests with start / goal node IDs or euclidean poses
- Service interface to change navigation route graphs at run-time
- Edge scoring dynamic plugins return a cost for traversing an edge and may mark an edge as invalid in current conditions
- Graph file parsing dynamic plugins allow for use of custom or proprietary formats
- Operation dynamic plugins to perform arbitrary tasks at a given node or when entering or leaving an edge on the route
- Operation may be graph-centric (e.g. graph file identifies operation to perform) or plugin-centric (e.g. plugins self-identify nodes and edges to act upon) 
- The nodes and edges metadata may be modified or used to communicate information across plugins including different types across different runs
- The Route Tracking action returns regular feedback on important events or state updates (e.g. rerouting requests, passed a node, triggered an option, etc)
- If rerouting occurs during Route Tracking along the previous current edge, that state will be retained for improved behavior and provide an interpolated nav_msgs/Path from the closest point on the edge to the edge's end (or rerouting's starting node) to minimize free-space planning connections where a known edge exists and is being continued.

## Design

TODO architectural diagram (needs final architecture hashed out)
TODO main elements of the package and their role (needs final architecture hashed out)

### Plugin Interfaces

TODO provide specifications for plugins // what they do // links (needs plugin API stability)
TODO plugins list

## Metrics

TODO provide analysis (needs completion + benchmark testing)

The use of Kd-trees to find the nearest start and goal nodes in the graph to the request is over 140x faster than data-structure lookups (0.35 ms/1000 lookups vs 50 ms/1000 lookups).

## Parameters

TODO provide full list (needs completion)

## File Formats

The graphs may be stored in one of the formats the parser plugins can understand or implement your own parser for a particular format of your interest!
A parser is provided for GeoJSON formats.
The only three required features of the navigation graph is (1) for the nodes and edges to have identifiers from each other to be unique for referencing and (2) for edges to have the IDs of the nodes belonging to the start and end of the edge and (3) nodes contain coordinates.
This is strictly required for the Route Server to operate properly in all of its features.

Besides this, we establish some requirements and conventions for route graph files to standardize this information to offer consistent and documented behavior.
The unique identifier for each node and edge should be given as `id`. The edge's nodes are `startid` and `endid`. The coordinates are given in an array called `coordinates`.
While technically optional, it is highly recommended to also provide:
- The node's frame of reference (`frame`), if not the global frame and you want it transformed
- The Operation's `trigger` (e.g. enter, exit edge, node achieved) and `type` (e.g. action to perform), as relevent

While optional, it is somewhat recommended to provide, if relevent:
- The edge's `cost`, if it is fixed or edge scoring plugins are not used
- Whether the edge's cost is `overridable` with edge scoring plugins, if provided

Otherwise, the Node, Edge, and Operations may contain other arbitrary application-specific fields with key-value pairs under the `metadata` key-name.
These can be primitive types (float, int, string, etc), vector types (e.g. a polygon or other vector of information), or even contain information nested under namespaces - whereas a metadata object may exist as a key's value within `metadata`.

While GeoJSON is not YAML-based, the following YAML file is provided as a more human-readable example for illustration of the conventions above.
Usable real graph file demos can be found in the `graphs/` directory.

```
example_graph.yaml

Node1:                     // <-- If provided by format, stored as name in metadata
  id: 1                    // <-- Required
  coordinates: [0.32, 4.0] // <-- Required
  frame: "map"             // <-- Highly recommended
  metadata:
    class: "living_room"   // <-- Metadata for node (arbitrary)
  operation:
    pause:                 // <-- If provided by format, stored as name in metadata
      type: "stop"         // <-- Required
      trigger: ON_ENTER    // <-- Required
      metadata:
        wait_for: 5.0      // <-- Metadata for operation (arbitrary)

Edge1:                     // <-- If provided by format, stored as name in metadata
  id: 2                    // <-- Required
  startid: 1               // <-- Required
  endid: 3                 // <-- Required
  overridable: False       // <-- Recommended
  cost: 6.0                // <-- Recommended, if relevent
  metadata:
    speed_limit: 0.85      // <-- Metadata for edge (arbitrary). Use abs_speed_limit if not a percentage
  operations:
    open_door:             // <-- If provided by format, stored as name in metadata
      type: "open_door"    // <-- Required
      trigger: ON_EXIT     // <-- Required
      metadata:
        door_id: 54        // <-- metadata for operation (arbirary)
        service_name "open-door"  // <-- metadata for operation (Recommended)
```

### Metadata Conventions for Convenience

While other metadata fields are not required nor necessarily needed, there are some useful standards which may make your life easier within in the Route Server framework.
These are default fields for the provided plugins, but you're free to embed this information anyway you like (but may come at the cost of needing to re-implement provided capabilities).
A set of conventions are shown in the table below. The details regarding the convention and where they are used within the codebase follows in more detail.

| Graph File Key  | Information Stored |
| --------------- | ------------- |
| `speed_limit`    | Speed limit, represented in percentage 0.0-1.0.  |
| `abs_speed_limit`    | Speed limit, represented in `m/s`.  |
| `penalty`    | A cost penalty to apply to a node or edge. |
| `class`    | The semanic class this node belongs to.  |
| `service_name`    | Which service to call, if node or edge should trigger an event.  |
| `abs_time_taken`    | Time taken to traverse an edge in `s`.  |


The `DistanceScorer` edge scoring plugin will score the L2 norm between the poses of the nodes constituting the edge as its unweighted score.
It contains a parameter `speed_tag` (Default: `speed_limit`) which will check the edge's metadata if it contains an entry for speed limit information, represented as a percentage of the maximum speed.
If so, it will adjust the score to instead be proportional to the time rather than distance.
Further, the `AdjustSpeedLimit` Route Operation plugin will utilize the same parameter and default `speed_limit` value to check each edge entered for a set speed limit percentage.
If present, it will request an adjusted speed limit by the controller server on each edge entered.
Thus, by convention, we say that `speed_limit` attribute of an edge should contain this information.

The `TimeScorer` plugin operates with an exact analog to the `DistanceScorer`, except rather than using speed limits based on percentages of maximum, it uses actual speed limits in `m/s`. As such the `speed_tag` parameter (Default: `abs_speed_limit`) is used to represent absolute speed limits in the metadata files. By convention, we say that the `abs_speed_limit` attribute of an edge should contain this information.

Similarly, the `penalty_tag` parameter (Default: `penalty`) in the `PenaltyScorer` can be used to represent a static unweighted cost to add to the sum total of the edge scoring plugins.
This is useful to penalize certain routes over others knowing some application-specific information about that route segment. Technically this may be negative to incentivize rather than penalize.
Thus, by convention, we say the `penalty` attribute of an edge should contain this information.

By convention, to use the `SemanticScorer` edge scoring plugin, we expect the semantic class of a node or edge to be stored in the `class` metadata key (though is reconfigurable to look at any using the `semantic_key` parameter). This can be used to store arbitrary semantic metadata about a node or edge such as `living_room`, `bathroom`, `work cell 2`, `aisle45`, etc which is then correlated to a set of additional class costs in the plugin's configuration. However, if the `semantic_key` parameter is set to empty string, then instead it checks **all keys** in the metadata of a node or edge if they match any names  of the semantic classes. This way, semantic information may be embedded as keys with other values (for another application) or as values themselves to the `class` key if only needing to specify its membership.

The Route Operation `TriggerEvent` and more broadly any operation plugins derived from `RouteOperationClient<SrvT>` (a service-typed template route operation base class to simplify adding in custom plugins based on service calls) relies on the parameter and matching metadata key `service_name` to indicate the service name to call with the corresponding route operation. When set in the parameter file, this will be used for all instances when called in the navigation route graph. When `service_name` is set in the operation metadata in the route graph, it can be used to specify a particular service name of that service type to use at that particular node/edge, created on the fly (when a conflict exists, uses the navigation graph as the more specific entry). 
That way both design patterns work for a Route Operation `OpenDoor` of service type `nav2_msgs/srv/OpenDoor`, for example:
- A `open_door/door1` (and `door2` and so on) service specific to each node containing a door to open may be called contextually and correctly at each individual door in the graph file metadata. This way you can have individual services (if desired) without having to have individual repetative operation plugin definitions. 
- A `open_doors` general service specified in the parameter file to call to open a door specified in the service's `request` field, so that one service is called for all instances of doors in the graph file without repetition in the graph file and storing client resources (just adding info about which one from the node metadata).

Thus, we say that `service_name` is a key to correspond to a string of the service's name to call in an operation to use `TriggerEvent` and `RouteOperationClient<SrvT>` plugins and base classes.

By convention, we reserve the edge metadata field `abs_time_taken` for representing actual navigation times along the edge. This is populated by the `TimeMarker` route operation plugin and utilized by the `TimeScorer` to more accurately represent the times to navigate along an edge based on the real execution time. While this is used as an internal mechanism to share data between live operations and route planning, it is also possible to set `last_time_taken` in your navigation graph file based on many execution runs to use in the `TimeScorer` to optimize system performance for a fleet or over long durations (and remove `TimeMarker` operation for live updates).

## Etc. Notes

### Metadata Communication

The metadata contained in the graph's nodes and edges can serve a secondary purpose to communicating arbitrary information from the graph file for use in routing behavior or operations. It may also be used to communicate or store information about a node or edge during run-time to query from a plugin in a future iteration, from another plugin in the system, or from another plugin type entirely.

For example: 
- If the collision monitor Route Operation identifies an edge as being blocked on a regular basis, a counter can be used to track the number of times this edge is blocked and if exceeding a threshold, it adds additional costs to that edge to incentivize taking another direction in an Edge Scorer. 
- If we want to minimize the time to traverse the space, rather than estimating the times to traverse an edge, we can store actual times to navigate into the metadata of the edges. This can be stored as part of a route operation after completing an edge and retrieved at planning time by an edge scorer.

All of this is made possible by the centralized graph representation and pointers back to its memory locations at each stage of the system.

### Node Achievement

The Route Tracker will track the progress of a robot following a defined route over time. When we achieve a node, that is to say, we pass it, that triggers events based on reaching a node (Also: exiting an old edge, entering a new edge). Thus, the specification of node achievement is worth some discussion for users so they can best use this powerful feature. 

The node achievement logic will first check if the robot is within a configurable radius of a node. This radius should be **generous** and not equatable to the goal tolerance. This should be sufficiently large that by the mechanics of your control, you can achieve a node when you pass it considering realistic deviations of path tracking by your trajectory planner. This may not need to be very large if using an exact path follower controller but may need to be relatively large for a dynamic obstacle avoidance planner. If in doubt, make it **larger** since this is merely the first stage for checking node achievement, not the metric itself.

Once we're within the range of a node that we need to consider whether or not we've achieved the node, we evaluate a mathematical operation to see if the robot has moved from the regime of the previous edge into the next edge spatially. We do this by finding the bisecting vector of the two edge vectors of interest (e.g. last edge and next edge) and comparing that with the distance vector from the node. When the dot product is 0, that means the we're at the orthogonal vector to the bisector moving from one regime to the next. Thus, we can check the sign of this dot product to indicate when we transition from one edge to the next while allowing for significant deviation from the node itself due to path tracking error or dynamic behavior.

For the edge boundary cases where there is no last edge (e.g. starting) or next edge (e.g. finishing), we use the boundary radius only (separate param from otherwise used radius). Recall that this only applies the routing part of the navigation request, the controller will still continue tracking the path until your goal achievement defined in your local trajectory planner's configurations. However, any Route Operations to be performed at the start or end nodes will be slightly preempted when compared to the others, by the choice of this radius.

A special case exists for rerouting, where as if we reroute along the same edge as we were previously routing through, the tracker state will be reloaded. While the current edge is not reported in the route message (because the last node is passed), that information is inserted back in the tracker to perform edge exit route operations -- and most importantly, the refined bisector-based node achievement criteria (which is why Steve went through the sheer pain to make it possible). 

---

# Steve's TODO list

- [ ] Test working full system + plugins + rerouting + pruning + operations + feedback / states proper
- [ ] Integration tests: tracking server + tracker state / feedback / operations correctness + completion. rerouting_start_id + Blocked ids?
  - Cases: reroute often OK. Reroute in same direction + path partial. Reroute off different direction + no path partial. Start route along first edge. edge exit event occurs at exit of partial + refined estimate.

- [ ] Sample files: AWS final
- [ ] QGIS demo + plugins for editing and visualizing graphs
- [ ] use map for checking start/goal nodes for infra blockages not just NN. Evaluate K. Share costmap?

- [ ] Quality: 
  - BT nodes for 2x route APIs + cancel nodes (+ groot xml + add to BT navlist + add to default yaml list),
  - web documentation (BT node configuration page, package configuration page),
  - readme complete,
  - Keep graph visualization in rviz but turn off by default
  - Default graph sandbox + launch configuration to set (and get filepath + mirror `map` comments). 

  - Tutorials (bt change, plugin customize, file field examples)
  - BT XMLs (first/last mile, freq. replanning, navigation using it, WPF using it, clearance)
  - Demos (route -> global -> local. outdoor non-planar. waypoint follower (GPS?) of route nodes. controller server of dense path)

- [ ] Testing by users

# Questions

- How can BT trigger BT node again if its still feedback-pending? preemption! Make sure works. + regular replanning and/or reroute BT node as a fallback to the controller server failing (add current edge to closed list?).
