# Tracking externally planned routes

`track_precomputed_route` accepts `nav2_msgs/action/TrackPrecomputedRoute`.
It executes an exact directed edge sequence against the server's loaded graph,
without calling the route planner, pruning the sequence, or rerouting locally.
This supports fleet planners that own route selection while Nav2 handles robot
progress, dense path generation, and graph operations.

The goal contains an opaque `route_id`, a `start_node_id`, and ordered `edge_ids`.
IDs are resolved from the current node's outgoing edges, preserving the server's
canonical coordinates, metadata, and operations. Unknown or disconnected edges
and ambiguous outgoing IDs fail with `INVALID_ROUTE`. Opposite directions may
reuse an edge ID; duplicate IDs among a node's outgoing edges are ambiguous.
Loops and repeated edges are preserved. An empty edge list tracks one node.
The reported route cost is the sum of stored edge costs; edge scorers do not run.

The robot must be within `boundary_radius_to_achieve_node` of the supplied start.
The caller must arrange the approach before submitting the goal. Missing robot
TF produces `TF_ERROR`. The route provider and server must use the same graph;
`route_id` identifies the request, not a graph version or a deduplication key.

Feedback echoes `route_id`, the route, the densified path, current edge and node
IDs, and triggered operations. Tracking does not drive the robot: consume the
path with `FollowPath`, as with `ComputeAndTrackRoute`. Cancellation stops
tracking; a replacement goal on the same action preempts the previous request.
Operations requesting a reroute abort with `REROUTE_REQUIRED` and their
`blocked_ids`, allowing the provider to submit a new route. IDs may be empty
when an operation requests replanning without identifying blocked elements.
Operational failures produce `OPERATION_FAILED`.

Only one tracking action may run at a time. A competing new tracking action
fails with `BUSY`; the existing `ComputeAndTrackRoute` action uses `UNKNOWN`
and a descriptive message because its existing interface has no busy code.
`ComputeRoute` remains available during tracking. Graph replacement through
`set_route_graph` returns `success=false` while any route request holds graph
references; retry after completion or cancellation.

The `TrackPrecomputedRoute` BT plugin uses these input ports:

```xml
<TrackPrecomputedRoute route_id="{fleet_route_id}" start_node_id="1"
                      edge_ids="10;20" path="{path}" route="{route}"
                      blocked_ids="{blocked_ids}"
                      error_code_id="{route_error_code}" error_msg="{route_error_msg}"/>
```

`edge_ids` also accepts a `std::vector<uint16_t>` blackboard value. An empty
string represents a single-node route. Updating any goal input preempts the
current request. `execution_duration`, node/edge IDs, and `operations_triggered`
are additional outputs. On termination, transient feedback outputs are cleared;
failure results retain their blocked IDs and error for the caller's recovery.
The existing compute actions and their local rerouting behavior remain available.
