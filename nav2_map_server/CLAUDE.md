# nav2_map_server — assistant guide

Lifecycle nodes serving / saving / costmap-filtering occupancy maps. Three executables ship from this package: `map_server`, `map_saver`, `costmap_filter_info_server`.

## What it does
- `map_server` — loads an `.yaml` + `.pgm`/`.png` map and publishes `OccupancyGrid` + `MapMetaData`.
- `map_saver` — saves the current map via the `SaveMap` service.
- `costmap_filter_info_server` — broadcasts filter info (mask, type, params) used by `nav2_costmap_2d` filter plugins.

## Layout
- `include/nav2_map_server/` — `map_server.hpp`, `map_saver.hpp`, `map_io.hpp`, `costmap_filter_info_server.hpp`
- `src/` — three node binaries + IO helpers
- `test/` — IO format tests + integration tests

## How to extend
- Support a new map image format → `map_io.cpp` (image read/write helpers).
- New filter info type → extend the message in `nav2_msgs` and the broadcaster here in lockstep.

## Tests
`colcon test --packages-select nav2_map_server`

## Docs to update
- `docs.nav2.org` → map-server tutorial and `configuration/packages/configuring-map-server.md`.

## Pitfalls
- Map YAML format is a public contract — additive fields only; old maps must keep loading.
- Saved maps must round-trip through `map_server` cleanly — regression-test with a saved-then-loaded map.
- Costmap filter mask + info topics are a paired contract with `nav2_costmap_2d` filter layer plugins; coordinate changes.
