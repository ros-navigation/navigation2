# Nav2 Route Server

The Route Server is a Nav2 Task server to compliment the Planner Server's free-space planning capabilities with pre-defined Navigation Route Graph planning, created by [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/) while at [Samsung Research America](https://www.sra.samsung.com/).
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
- Action interface request can process requests with start / goal node IDs or poses
- Service interface to change navigation route graphs at run-time
- Edge scoring dynamic plugins return a cost for traversing an edge and may mark an edge as invalid in current conditions
- Common edge scoring plugins are provided for needs like optimizing for distance, time, cost, static or dynamic penalties
- Graph file parsing dynamic plugins allow for use of custom or proprietary formats
- Provided 2 file parsing plugins based on popular standards: OpenStreetMap and GeoJSON
- Operation dynamic plugins to perform arbitrary tasks at a given node or when entering or leaving an edge on the route (plan)
- Operation may be graph-centric (e.g. graph file identifies operation to perform) or plugin-centric (e.g. plugins self-identify nodes and edges to act upon) 
- TODO Operations provided (needs Operations implemented)
- TODO arbitary metadata storage that can be communicated between plugins or even classes of plugins about nodes, edges: Time taken
- TODO action feedback on updates (reroute, operaiton triggered, state change passing a node/edge)

## Design

TODO architectural diagram (needs final architecture hashed out)
TODO main elements of the package and their role (needs final architecture hashed out)

### Plugin Interfaces

TODO provide specifications for plugins // what they do // links (needs plugin API stability)

## Metrics

TODO provide analysis (needs completion)

The use of Kd-trees to find the nearest start and goal nodes in the graph to the request is over 140x faster than the brute force data-structure lookup (0.35 ms/1000 lookups vs 50 ms/1000 lookups).

## Parameters

TODO provide full list (needs completion)

## File Formats

The graphs may be stored in one of the formats the parser plugins can understand or implement your own parser for a particular format of your interest!
Parsers are provided for OpenStreetMap (OSM) and GeoJSON formats.
The only two required features of the navigation graph is for the nodes and edges to have identifiers from each other to be unique for referencing and for edges to have the IDs of the nodes belonging to the start and end of the edge.
This is strictly required for the Route Server to operate properly in all of its features.

Besides this, we establish some requirements and conventions for route graph files _for the provided parsers_ to standardize this information.
These conventions are utilized by the provided parsers to offer consistent and documented behavior. 
These are also special values stored in the graph nodes and edges outside of the arbitrarily defined metadata.

In the provided parsers, the unique identifier for each node and edge should be given as `id`. The edge's nodes are `startid` and `endid`.
While technically optional, it is highly recommended to also provide:
- The node's location and frame of reference (`x`, `y`, `frame`)
- The Operation's `trigger` (e.g. enter, exit edge, node achieved) and `type` (e.g. action to perform)

While optional, it is somewhat recommended to provide, if relevent:
- The edge's `cost`, if it is fixed or edge scoring plugins are not used
- Whether the edge's cost is `overridable` with edge scoring plugins, if provided

Otherwise, the Node, Edge, and Operations may contain other arbitrary application-specific fields with key-value pairs.
The keys are stored as strings in `metadata` within the objects which acts as a python3 dict (e.g. `std::unordered_map<std::string, std::any>`) with an accessor function `T getValue(const std::string & key, T & default_val)` to obtain the field's values within all plugins.

While neither OSM nor GeoJSON are YAML-based, the following YAML file is provided as a more human-readable example for illustration of the conventions above.
Example files in both formats can be found TODO LINKS TO EXAMPLE FILES

```
example_graph.yaml

Node1:                  // <-- If provided by format, stored as name in metadata
  id: 1                 // <-- Required
  x: 0.32               // <-- Highly recommended
  y: 4.3                // <-- Highly recommended
  frame: "map"          // <-- Highly recommended
  class: "living_room"  // <-- Metadata for node (arbitrary)
  operation:
    pause:              // <-- If provided by format, stored as name in metadata
      type: "stop"      // <-- Required
      trigger: ON_ENTER // <-- Required
      wait_for: 5.0     // <-- Metadata for operation (arbitrary)

Edge1:                  // <-- If provided by format, stored as name in metadata
  id: 2                 // <-- Required
  startid: 1            // <-- Required
  endid: 3              // <-- Required
  speed_limit: 0.85     // <-- Metadata for edge (arbitrary). Use abs_speed_limit if not a percentage
  overridable: False    // <-- Recommended
  cost: 6.0             // <-- Recommended, if relevent
  operations:
    open_door:          // <-- If provided by format, stored as name in metadata
      type: "open_door" // <-- Required
      trigger: ON_EXIT  // <-- Required
      door_id: 54       // <-- metadata for operation (arbirary)
      service_name "open-door"  // <-- metadata for operation (Recommended)
```

### Metadata Conventions for Convenience

TODO have a tl;dr starting before explaining

While other metadata fields are not required nor necessarily needed, there are some useful standards which may make your life easier within in the Route Server framework.
These are default fields which if used can make your life easier with the provided plugins, but you're free to embed this information anyway you like (but may come at the cost of needing to re-implement provided capabilities).

The `DistanceScorer` edge scoring plugin will score the L2 norm between the poses of the nodes constituting the edge as its unweighted score.
This uses the node's location information as "highly recommended" above.
It contains a parameter `speed_tag` (Default: `speed_limit`) which will check `edge.metadata` if it contains an entry for speed limit information, represented as a percentage of the maximum speed.
If so, it will adjust the score to instead be proportional to the time rather than distance.
Further, the `AdjustSpeedLimit` Route Operation plugin will utilize the same parameter and default `speed_limit` value to check each edge entered for a set speed limit percentage.
If present, it will request an adjusted speed limit by the controller server on each edge entered.
Thus, by convention, we say that `speed_limit` attribute of an edge should contain this information.

The `TimeScorer` plugin operates with an exact analog to the `DistanceScorer`, except rather than using speed limits based on percentages of maximum, it uses actual speed limits in `m/s`. As such the `speed_tag` parameter (Default: `abs_speed_limit`) is used to represent absolute speed limits in the metadata files. By convention, we say that the `abs_speed_limit` attribute of an edge should contain this information.

Similarly, the `penalty_tag` parameter (Default: `penalty`) in the `PenaltyScorer` can be used to represent a static unweighted cost to add to the sum total of the edge scoring plugins.
This is useful to penalize certain routes over others knowing some application-specific information about that route segment. Technically this may be negative to incentivize rather than penalize.
Thus, by convention, we say the `penalty` attribute of an edge should contain this information.

By convention, to use the `SemanticScorer` edge scoring plugin, we expect the semantic class of a node or edge to be stored in the `class` metadata key (though is reconfigurable to look at any). This can be used to store arbitrary semantic metadata about a node or edge such as `living_room`, `bathroom`, `work cell 2`, `aisle45`, etc which is then correlated to a set of additional class costs in the `SemanticScorer` configuration to add additional costs to different types of distinct areas as semantically relevent to an application. However, if the `semantic_key` parameter is set to empty string, then instead it checks **all keys** in the metadata of a node or edge if they match any names  of the semantic classes. This way, semantic information may be embedded as keys with other values (for another application) or as values themselves to the `class` key if only needing to specify its membership.

The Route Operation `TriggerEvent` and more broadly any operation plugins derived from `RouteOperationClient<SrvT>` (a service-typed template route operation base class to simplify adding in custom plugins based on service calls) relies on the parameter and matching operation key `service_name` to indicate the service name to call with the corresponding route operation. When set in the parameter file, this will be used for all instances when called in the navigation route graph. When `service_name` is set in the operation in the navigation route graph, it can be used to specify a particular service name of that service type to use at that particular node/edge, created on the fly (when a conflict exists, uses the navigation graph as the more specific entry). 
That way both design patterns work for a Route Operation `OpenDoor` of service type `nav2_msgs/srv/OpenDoor`, for example:
- A `open_door/door1` (and `door2` and so on) service specific to each node containing a door to open may be called contextually and correctly at each individual door in the graph file. This way you can have individual services (if desired) without having to have individual repetative operation plugin definitions. 
- A `open_doors` general service specified in the parameter file to call to open a door specified in the service's `request` field, so that one service is called for all instances of doors in the graph file without repetition in the graph file and storing client resources (just adding info about which one from the node metadata).

Thus, we say that `service_name` is a key to correspond to a string of the service's name to call in an operation to use `TriggerEvent` and `RouteOperationClient<SrvT>` plugins and base classes.

By convention, we reserve the edge metadata field `abs_time_taken` for representing actual navigation times along the edge. This is populated by the `TimeMarker` route operation plugin and utilized by the `TimeScorer` to more accurately represent the times to navigate along an edge based on the real execution time. While this is used as an internal mechanism to share data between live operations and route planning, it is also possible to set `last_time_taken` in your navigation graph file based on many execution runs to use in the `TimeScorer` to optimize system performance for a fleet or over long durations (and remove `TimeMarker` operation for live updates).

## Etc. Notes

TODO Document metadata can be used to communicate from the operations back to the planning algorithm (this is blocked, this is the new time/dist to use based on actual previous executions) to refine operations over time or just communicate information about the state of the graph to each other as choosing. This is due to pointers instead of copies of the graph around the system. Example use in the time scorer edge scoring plugin and the time marker route operation.

TODO document generous window b/c only used as a pre-condition + want dynamic behavior // refinement to make pretty exact (show diagram of method). also in param description. 

---

# Steve's TODO list

- [ ] live route analyzer working + plugins + rerouting + pruning start at reroute

- [ ] if start/ending pose is between 2 nodes in the graph solution (prune first) -- in server

- [ ] way to clear aggrevated blocked routes other than preemption/new request? As a fallback if route cnanot be found except current cycles'? Max retries? but what if still blocked? cause back and forth






- [ ] Create basic file format for graph + parser. Vector types (regions), recursion namespaces, structured types? Operations/metadata.
- [ ] QGIS demo + plugins for editing and visualizing graphs



- [ ] Quality: BT nodes, Python API, testing (server, tracker), documentation, tutorial (bt change, plugin customize, file field examples). BT XML for first/last mile, freq. replanning, navigation using it, WPF, de-stucking? 

- [ ] centralized 1 costmap to pass aroun dfor use (if possible): edge score, operation, selecting start/goal nodes with infra
- [ ] use map for checking start/goal nodes for infra blockages not just NN. Evaluate K 
- [ ] cleanup code where necessary (large functions, etc) and optimize performance (profile), lots of Result and State and Return objects of similar name and contents. Refine names / replace some?

- [ ] demos with route -> global -> local. outdoor non-planar. to waypoint follower (GPS?) of nodes.

# Questions

- How can BT trigger BT node again if its still feedback-pending? preemption! Make sure works. + regular replanning and/or reroute BT node as a fallback to the controller server failing (add current edge to closed list?).
- How to stop / pause robot in the server and/or the behavior tree? Clearance stuff
