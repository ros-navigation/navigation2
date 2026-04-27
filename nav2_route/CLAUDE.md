# nav2_route — assistant guide

Higher-level **graph/route planner** lifecycle node. Loads a GeoJSON graph, finds a route between graph nodes, and exposes route progress so a controller can follow it. **Kilted-only** — never backport to Humble or Jazzy.

## Plugin interfaces (all in `nav2_route/include/nav2_route/`)
- `EdgeScorer` plugins — score edges during search.
- `Operation` plugins — actions triggered when a node/edge is reached (e.g. open door, slow down).
- `RouteSearch` — pluggable search algorithm.
- `GoalIntentExtractor` — derives goal nodes from a `geometry_msgs::PoseStamped`.

## Plugin XML
- `plugins.xml` — registers shipped scorers, operations, search variants.

## Layout
- `include/nav2_route/` — engine, scorers, operations, search algorithms
- `src/` — server + plugin implementations
- `graphs/` — example GeoJSON graphs (canonical schema reference)
- `test/`

## How to add a new edge scorer / operation
1. Header in `include/nav2_route/`, derive from the relevant base.
2. `PLUGINLIB_EXPORT_CLASS(...)` against the right base class.
3. Add `<class>` block to `plugins.xml`.
4. Register in the route server param block in `nav2_bringup/params/nav2_params.yaml`.

## Tests
`colcon test --packages-select nav2_route`

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-route-server.md` and route concepts pages.
- Plugins page on new scorer / operation.
- Graph file schema docs if the GeoJSON contract changes.

## Pitfalls
- Kilted-only — `package.xml` build deps are not available on Humble/Jazzy. Don't add backport tags.
- GeoJSON graph schema is part of the contract with users — additive changes only; deprecate properties before removal.
- Operations can fire mid-route — they must be non-blocking and idempotent.
