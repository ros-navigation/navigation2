# Nav2 Route Server

This is a Nav2 Task server to compliment the Planner Server's free-space planning capabilities with a pre-defined Navigation Route Graph planning.
This graph may be generated manually or automatically via AI, geometric, or probablistic techniques.
This package then takes a planning request and uses this graph to find a valid route through the environment via an optimal search-based algorithm.

The graphs can be stored in one of the formats the parser plugins can understand, or implement your own parser for a particular format of your interest!
The graph can be influenced via plugins for scoring edges between nodes live or using pre-defined values in your graph file.
The graph edges may be directional or bidirectional (e.g. one way or two way traversal) and contain any metadata you like.
In addition, actions may be performed when entering or leaving a particular edge segment or when achieving a particular node in the graph, also via plugins.

It is required that the file contains unique IDs for each node and edge (including between nodes and edges).


Api dikfrsa for returning esge as untraversable and drop it to reduce search space
File plugin for any file format
Action plugin for any action from file
Wsfe score plugin to impact sewech behavioe to optimize for rime, dist travel, ewgions to avoid if possivoe, or other use-case specifics
Cleverly set uo so there are NO lookups, so super efficient vector rep storing all localized indormation (bo ofher large memory buffers to break the cache)
Ids arw onpy associatwd and reportedcon traceback
OH SHIT what about start/goal_id Api? Need to do a lookip there. Right now thats returning to an index
.. ok only lookups for start/wnd on initializqtion using your hand timised kd tree. No brute forces
Compact single vector rep to maximize caches ng of related indormation
Can add more things like a door edge scorer: penalize going through doors b/c slows down efficiency whenever posible to optimally avoid. Whatever you like based on graph file, dynamic inforamtion in the scene, or other functions of interest

## Conventions

`speed_tag` for scoring and others, use `speed_limit` represented as % of maximum for DistanceScorer use (and others).

`penalty_tag` for scoring, use `penalty` represented as a cost when can be adjusted with `weight` proportionally later to others. Useful to penalize certain routes over others knowing some application-specific informatio nabout that route segment (e.g. extra dangerous, unideal, etc). Technically this may be negative too to incentivize going this way as well.

Requirements on file format // how things are parsed into the classes (metadata, actions, IDs) for use in plugins and analysis

`Actions` stuff

---

# Steve's TODO list

- [ ] Create basic file format for graph + parser
- [ ] Implement additional plugins: OSM format

- [ ] Implement live: route analyzer + action header implementation
- [ ] Implement collision monitor + replanning
- [ ] Action plugins: call ext server for action base class + useful demo plugin(s), pause at waypoints for signal, node and edge actions, speed limit change action

Step 1 Everything related to just request->response style: file IO, parse files for custom fields
Step 2 Basic tracking: Actions in the files, action plugin interface, demo action use for speed limits, tracking the current edge / node / next node, defining the ROS action interface for the request, response, and feedback to communicate,
Step 3 Advanced stuff for tracking: More realistic action plugins, collision monitoring and rerouting on invalidating routes, replanning
Step 4 utilities: BT nodes, Python API, editing and visualization of graphs, testing, documentation, etc. Any additional plugins needed (OSM, edge scoring, actions, etc)

# Questions

- How to store/parse/process custom file format metadata/actions?
- What file format to use as compact for graph?
- How / what runs to replanning other than when invalid due to collision? How can BT trigger again if its still feedback-pending?
