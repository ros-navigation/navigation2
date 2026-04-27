# nav2_costmap_2d — assistant guide

Provides the **costmap library** (`Costmap2DROS`, `LayeredCostmap`, `Costmap2D`) used by every server, plus ships the default **Layer plugins** (static, obstacle, voxel, inflation, range, denoise, plugin-container). Cross-cutting infrastructure — touch carefully.

## Interface
`nav2_costmap_2d::Layer` — `include/nav2_costmap_2d/layer.hpp`. Two key callbacks: `updateBounds(robot_x, robot_y, robot_yaw, *min_x, *min_y, *max_x, *max_y)` and `updateCosts(master_grid, min_i, min_j, max_i, max_j)`.

## Plugin XML
- `costmap_plugins.xml` — registers Layer + Filter plugins.

## Layout
- `include/nav2_costmap_2d/` — core library (`costmap_2d_ros.hpp`, `costmap_2d.hpp`, `layered_costmap.hpp`, `layer.hpp`, `costmap_subscriber.hpp`, `clear_costmap_service.hpp`)
- `include/nav2_costmap_2d/<layer>_layer.hpp` — built-in layer headers
- `src/` — core lib + layer implementations
- `plugins/` — additional plugin sources (e.g. costmap filters)
- `test/` — large test suite (`test_costmap_2d.cpp`, per-layer tests, integration tests)

## How to add a new layer
1. Header in `include/nav2_costmap_2d/<name>_layer.hpp` deriving from `nav2_costmap_2d::Layer` (or `CostmapLayer` for shared helpers).
2. Override `onInitialize`, `updateBounds`, `updateCosts`, `matchSize`, `reset`, `isClearable`.
3. `PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::<Name>Layer, nav2_costmap_2d::Layer)`.
4. Add `<class>` block to `costmap_plugins.xml`.
5. Register under the relevant costmap's `plugins` list in `nav2_bringup/params/nav2_params.yaml`.

## Tests
`./build/nav2_costmap_2d/test_<name>`
`colcon test --packages-select nav2_costmap_2d`

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-costmaps.md`, plus per-layer pages.
- Plugins page on a new layer / filter.
- Migration guide for any change to the `Layer` interface or `Costmap2DROS` public API — every server transitively depends on this.

## Pitfalls
- `updateBounds` must only **expand** the master bounds — never shrink. Forgetting to update `*max_*` to the layer's footprint causes silent staleness.
- `updateCosts` writes into `master_grid` — respect the `min_i/min_j/max_i/max_j` window. Out-of-bounds writes corrupt other layers.
- Layer state allocated in `onInitialize` must survive `matchSize` / `reset` calls — these can fire repeatedly during reconfigure.
- `isClearable` controls whether a layer is wiped by the `ClearCostmap` service — set it correctly per-layer or runtime clears do nothing useful.
- `LayeredCostmap` order matters: static → obstacle/voxel → inflation. A new layer's spot in the list is config-driven; document the recommended position.
