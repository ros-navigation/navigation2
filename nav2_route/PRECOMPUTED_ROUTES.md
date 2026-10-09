# Tracking externally planned routes

`track_precomputed_route` accepts `nav2_msgs/action/TrackPrecomputedRoute`.
The goal is an existing `nav2_msgs/msg/Route`: ordered nodes and directed edges
from the server's loaded graph. It must contain one more node than edges;
a single node with no edges is valid. Loops and repeated nodes or edges retain
their supplied order. No local planning, pruning or deduplication is performed.

The server resolves node IDs with its existing graph ID map, then resolves each
edge ID among the current node's outgoing edges to the next supplied node.
The loaded graph supplies coordinates, metadata, operations and stored costs;
message coordinates and cost do not override it. An empty frame is accepted;
a supplied frame must match `route_frame`. Unknown IDs, inconsistent sequences,
disconnected edges or multiple edges with the same ID and directed endpoints
fail with `NO_VALID_ROUTE`. Opposite directions may reuse an edge ID.

The robot must be within `boundary_radius_to_achieve_node` of the supplied start.
The caller arranges the approach before submitting a goal. Missing robot TF
produces `TF_ERROR`. Provider and server must use the same graph; graph version
negotiation is outside this interface.

The action UUID already identifies the request, so no separate correlation ID
is needed. Feedback follows `ComputeAndTrackRoute`: canonical route, densified
path, node and edge IDs, and triggered operations. Tracking does not drive the
robot: consume the path with `FollowPath`. Cancellation stops tracking;
replacement goals on the same action preempt the previous request.

An operation requesting rerouting aborts with `REROUTE_REQUIRED` instead of
selecting a different route locally. Its `blocked_ids` are retained as uint32
because operations use unsigned graph IDs and the external provider needs
those constraints to replan. They may be empty for a general reroute request.
These are discovered during execution, not available from the input route.
`execution_duration` follows the existing tracking action's result convention.
Operation failures produce `OPERATION_FAILED`.

The three actions share request validation, preemption, route publication,
path generation, exception handling and result completion. Both tracking
actions use the same tracker and feedback implementation. The computed action
continues to replan locally when operations request it.

Action execution uses independent worker threads. A graph read lock protects
raw graph pointers until each request exits; graph replacement returns
`success=false` while they are in use. One tracking lock protects shared tracker
and operation state: a competing external tracking action fails with `BUSY`,
while the existing computed tracking action uses `UNKNOWN` and a descriptive
message. A planning lock protects shared goal extraction, scorer and node search
state for the two compute actions. It is released before tracking, so
`ComputeRoute` remains available during either tracking action.

The BT plugin accepts a Route blackboard value:

```xml
<TrackPrecomputedRoute route="{fleet_route}" path="{path}"
                      route_feedback="{canonical_route}"
                      blocked_ids="{blocked_ids}"
                      error_code_id="{route_error_code}" error_msg="{route_error_msg}"/>
```

Updating `route` preempts the current request. `execution_duration`, node/edge
IDs and `operations_triggered` are additional outputs. Terminal states clear
transient feedback outputs; failures retain blocked IDs and error information.
