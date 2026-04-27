# nav2_behavior_tree — assistant guide

Provides the BT engine (BehaviorTree.CPP wrapper) and ships **all default BT nodes**: action, condition, control, decorator. Loaded by `nav2_bt_navigator` to compose runtime behavior. Touching a BT node almost always means touching downstream tree XML and docs.

## What ships here
- BT node base classes: `BtActionNode<>`, `BtServiceNode<>`, `BtCancelActionNode<>` etc. in `include/nav2_behavior_tree/`.
- Plugin trees:
  - `plugins/action/` — action client BT nodes (`compute_path_to_pose`, `follow_path`, `spin`, `back_up`, etc.)
  - `plugins/condition/` — predicates (`goal_reached`, `is_battery_low`, `path_expiring_timer`, etc.)
  - `plugins/control/` — control flow (`pipeline_sequence`, `recovery_node`, `round_robin_node`)
  - `plugins/decorator/` — decorators (`rate_controller`, `distance_controller`, `goal_updater`)

## Plugin XML
- `nav2_tree_nodes.xml` — **single index** of every shipped BT node. Used by Groot for graphical authoring.

## How to add a new BT node
1. Pick the right plugin family (action / condition / control / decorator). Header in `include/nav2_behavior_tree/plugins/<family>/`, impl in `plugins/<family>/`.
2. Derive from the matching base — for action nodes use `BtActionNode<ActionT>` and override `on_tick`, `on_success`, `on_aborted`, `on_cancelled`, `providedBasicPorts`.
3. At the bottom of the `.cpp`, `BT_REGISTER_NODES` factory function with `factory.registerNodeType<ClassName>("NodeName")`.
4. Add a corresponding `<Action ID="...">` (or Condition/Control/Decorator) entry to `nav2_tree_nodes.xml` listing the input/output ports.
5. Update the package readme node table (`README.md` and the BT library node lists per the PR template's BT-node checklist).
6. Add a unit test under `test/plugins/<family>/`.
7. If the node should be in a default tree, edit XMLs under `nav2_bt_navigator/behavior_trees/`.

## Tests
`./build/nav2_behavior_tree/test_<name>`
`colcon test --packages-select nav2_behavior_tree`

## Docs to update — BT-specific (PR template requires all)
- `docs.nav2.org` → `behavior_trees/index.md` and the node reference page.
- `nav2_tree_nodes.xml` (Groot index) — must list every shipped node with port specs.
- BT package readme node table.
- BT library lists referenced by the PR template.

## Pitfalls
- Port names are part of the public BT API — renaming a port silently breaks every downstream tree XML.
- `BtActionNode::on_tick` runs at tree tick rate — don't block. Use `RUNNING` while the action is in flight.
- Always implement `on_cancelled` — a parent recovery branch can preempt at any tick.
- Default BT XMLs in `nav2_bt_navigator/behavior_trees/` are part of the contract — coordinate edits there with this package.
