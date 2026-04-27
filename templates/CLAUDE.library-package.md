# <PKG_NAME> — assistant guide

<one-line role: "Shared library used by <consumers>; not a plugin host.">

## Public surface
- Headers in `include/<pkg>/` — every header here is part of the package's public API.
- Changes to public symbols are **API-breaking** and must go into the migration guide on docs.nav2.org.

## Layout
- `include/<pkg>/` — public headers
- `src/` — implementation (compiled into one or more libraries declared in `CMakeLists.txt`)
- `test/` — gtest binaries

## How to add / change
- New utility → add the header under `include/<pkg>/`, the impl under `src/`, register in `CMakeLists.txt` (target_sources / install).
- Modifying an existing header → check downstream consumers across the workspace before changing the signature; mass-rebuild with `colcon build` afterward.

## Tests
- `./build/<pkg>/<test_name>`
- `colcon test --packages-select <pkg>`

## Docs to update
- `docs.nav2.org` migration guide for any public-API change.
- Doxygen comments on every new public function.

## Pitfalls
- This package is consumed by ~all of nav2; one ABI change cascades into every dependent build. Prefer additive changes; deprecate before removing.
