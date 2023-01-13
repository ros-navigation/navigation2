# Nav2 Route Server

This is a Nav2 Task server to compliment the Planner Server's free-space planning capabilities with a pre-defined Navigation Route Graph planning.
This graph may be generated manually or automatically via AI, geometric, or probablistic techniques.
This package then takes a planning request and uses this graph to find a valid route through the environment via an optimal search-based algorithm.

The graphs can be stored in one of the formats the parser plugins can understand, or implement your own parser for a particular format of your interest!
The graph can be influenced via plugins for scoring edges between nodes live or using pre-defined values in your graph file.
The graph edges may be directional or bidirectional (e.g. one way or two way traversal) and contain any metadata you like.
In addition, actions may be performed when entering or leaving a particular edge segment or when achieving a particular node in the graph, also via plugins.

It is required that the file contains unique IDs for each node and edge (including between nodes and edges).

## Conventions

`speed_tag` for scoring and others, use `speed_limit` represented as % of maximum for DistanceScorer use (and others).

Requirements on file format // how things are parsed into the classes (metadata, actions, IDs) for use in plugins and analysis

`Actions` stuff

---

# Steve's TODO list

- [ ] GPS support
- [ ] Create basic file format for graph + parser


- [ ] Implement additional plugins: costmap blockage, OSM format,

- [ ] Implement live: route analyzer, Speed limit parse/edge action, call ext server for action base class + demo plugin, pause parse/node action
- [ ] Implement collision monitor + replanning


# Questions

- How to store/parse/process custom file format metadata/actions?
- What file format to use as compact for graph?
- How / what runs to replanning other than when invalid due to collision? How can BT trigger again if its still feedback-pending?
