# Nav2 Route Server

The Route Server is a Nav2 Task server to compliment the Planner Server's free-space planning capabilities with pre-defined Navigation Route Graph planning, created by [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/) while at [Samsung Research America](https://www.sra.samsung.com/).
This graph has few rules associated with it and may be generated manually or automatically via AI, geometric, or probablistic techniques.
This package then takes a planning request and uses this graph to find a valid route through the environment via an optimal search-based algorithm.
It may also live monitor and analyze the route's process to execute custom behaviors on entering or leaving edges or achieving particular graph nodes.

There are plugin interfaces throughout the server to enable a great deal of application-specific customization:
- Custom search-prioritization behavior with edge scoring plugins (e.g. minimize distance or time, mark blocked routes, enact static or dynamic penalties for danger and application-specific knowledge, etc)
- Custom actions to perform during route execution, triggered when entering or leaving an edge or achieving a graph node on the path (e.g. open door, pause at node to wait for clearance, adjust maximum speed, etc)
- Parsers of navigation graph files to use any type of format desirable (e.g. geoJSON, OpenStreetMap)

Additionally, the parsers store **additional arbitrary metadata** specified within the navigation graph files to store information such as speed limits, added costs, actions to perform.
Thus, we do not restrict the data that can be embedded in the navigation route graph for an application and this metadata is communicated to the edge scoring and action plugins to adjust behavior.
Note however that plugins may also use outside information from topics, services, and actions for dynamic behavior or centralized knowledge sharing as well. 

## Features

- Optimized Dikjstra's planning algorithm modeled off of the Smac Planner A* implementation
- Cleverly designed for no run-time lookups on the graph during search (2 lookups to find the start and goal edges on request initialization)
- Use of Kd-trees for finding the nearest node to arbitrary start and goal poses in the graph, over 140x faster than brute force search
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
- Action dynamic plugins to perform arbitrary tasks at a given node or when entering or leaving an edge on the route (plan)
- TODO action provided (needs actions implemented)

## Design

TODO architectural diagram (needs final architecture hashed out)
TODO main elements of the package and their role (needs final architecture hashed out)

### Plugin Interfaces

TODO provide specifications for plugins // what they do // links (needs plugin API stability)

## Metrics

TODO provide analysis (needs completion)

## Parameters

TODO provide full list (needs completion)

## File Formats

Parsers are provided for OpenStreetMap (OSM) and GeoJSON formats.
The graphs may be stored in one of the formats the parser plugins can understand or implement your own parser for a particular format of your interest!

The only required feature of the navigation graph is for the nodes and edges to have identifiers from each other to be unique for referencing.
In the provided parsers, this should be given as `id` for both edges and nodes.
While all other elements are optional, it is highly recommended to also provide the nodes' location and frame of reference of that information.
These are used in the default edge scoring plugin to compute the distance between nodes as the traversal cost (though this can be replaced with others if you choose).
In the provided parsers, these fields are TODO (needs Josh's parsers)

Otherwise, the node and edge objects may contain other fields with key-value pairs.
The keys are stored as strings in the `metadata` object within the nodes and edges which acts as a python3 dict (e.g. `std::unordered_map<std::string, std::any>`) with an accessor function `T getValue(const std::string & key, T & default_val)` to obtain the field's values within all plugins.

TODO how actions  (needs actions prototype)
TODO provide example files in both formats + links to them (needs Josh's parsers)

### Conventions for Convenience (though not required)

While other fields are not required nor necessarily highly recommended, there are some useful standards which may make your life easier and provide some inspiration for development of plugins in the Route Server framework.
These are default fields and parameters which can make your life easier, but you're free to embed this information anyway you like (but may come at the cost of needing to implement some redundant capabilities as provided plugins).

For edges with meaningful non-zero statically set costs, use the field `cost` in the edge to communicate this information in the form of a `float`.
If not otherwise specified, this may be overridden by Edge Scoring plugins, if they are provided.
If you would like this particular node to not to be overrided (e.g. this value is used, regardless of plugins), set a field `overridable` to `false` and this value will be used.

The `DistanceScorer` edge scoring plugin will come the L2 norm between the poses of the nodes constituting the edge as its unweighted score.
It contains a parameter `speed_tag` (Default: `speed_limit`) which will check `edge.metadata` if it contains an entry for speed limit information, represented as a percentage of the maximum speed.
If so, it will adjust the score to instead be proportional to the time rather than distance.
By convention, we say that `speed_limit` attribute of an edge should contain this information, but the parameter may be adjusted to use others.

Similarly, the `penalty_tag` parameter (Default: `penalty`) in the `PenaltyScorer` can be used to represent a static unweighted cost to add to the sum total of the edge scoring plugins.
This is useful to penalize certain routes over others knowing some application-specific information about that route segment. Technically this may be negative to incentivize rather than penalize.
By convention, we say the `penalty` attribute of an edge should contain this information, but likewise may be adjusted to use other tags via the parameter.

TODO `Actions` stuff (needs live monitoring prototype)

## Etc. Notes

---

# Steve's TODO list

- [ ] Unit testing: planner and server
- [ ] Dynamic scoring in the closed edges scroer

- [ ] Create basic file format for graph + parser: OSM and geoJSON

STEVE: do actions need any kind of specialized fields or anything? could still just be generic metadata nad let the plugins idenifty themselves if theyr'e valid for a given node/edge metadata set. But could set standard for "action" field just to have some way to consistenyly communicate it. But s till in the metadata class
STEVE: metadata class handle nested inforamtion?

- [ ] Implement live route analyzer: tracking the current edge / node / next node + action plugin header(s) + actions in graph files parsing / storage in classes + feedback action interface
- [ ] Action plugins: call ext server for action base class + useful demo plugin(s) - realistic, pause at waypoints for signal, node and edge actions, speed limit change action
- [ ] Implement collision monitor + replanning / rerouting on invalidation for other reasons (requested, time, plugin?)

- [ ] Quality: BT nodes, Python API, editing and visualization of graphs, testing, documentation, tutorial (bt change, plugin customize, file field examples)

# Questions

- How can BT trigger BT node again if its still feedback-pending?
- service for closed edge // topic for costmaps come in during request?

