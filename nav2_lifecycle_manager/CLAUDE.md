# nav2_lifecycle_manager — assistant guide

Drives the `configure → activate → deactivate → cleanup → shutdown` transitions across **all** Nav2 lifecycle nodes in a configured order. Anything that misbehaves on transitions hangs the entire stack — this package's correctness is load-bearing.

## What it does
- Reads ordered `node_names` list from params.
- Calls `change_state` services on each, in order, with timeouts.
- Exposes `manage_nodes` and `is_active` services for orchestration scripts and bringup.
- Optionally provides a watchdog (`bond` connections) — when a managed node dies, the manager deactivates the rest.

## Layout
- `include/nav2_lifecycle_manager/` — `lifecycle_manager.hpp`, `lifecycle_manager_client.hpp`
- `src/` — manager + main + the client used by other tooling
- `test/` — service-level tests

## How to change behavior
- Add a new managed phase (e.g. a new lifecycle subsystem) → extend the param schema and update `nav2_bringup/launch/` to populate `node_names`.
- New service → declare in `nav2_msgs`, expose via the manager.

## Tests
`colcon test --packages-select nav2_lifecycle_manager`

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-lifecycle.md`.
- Migration guide for any change to managed-service signatures.

## Pitfalls
- Order matters: costmap-owning servers must be configured before their consumers. Bad order = hang.
- Timeouts that are too short on slow CI hosts make CI flaky; tune via `bond_timeout` / `service_timeout` params, not by removing nodes.
- The bond watchdog can mask root causes — when debugging hangs, capture the failing transition's status before letting the watchdog cascade-deactivate.
