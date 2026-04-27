# nav2_common — assistant guide

Build-time helpers used by **every** package in the stack:

- `nav2_package()` CMake macro — applies project-wide compile flags, C++17, linker settings. Every `CMakeLists.txt` in the repo calls it.
- Python helpers for launch substitution and parameter-file rewriting (used by `nav2_bringup` launch files).
- mypy / lint configuration helpers.

## Layout
- `cmake/` — CMake macros (incl. `nav2_package.cmake`)
- `nav2_common/` — Python helpers
- `package.xml` declares ament_cmake build type with Python install

## How to change `nav2_package()`
1. Edit `cmake/nav2_package.cmake`.
2. **Rebuild the entire workspace**, not just this package — every downstream `CMakeLists.txt` re-evaluates the macro.
3. If flags become stricter, audit downstream packages for new warnings before merging.

## Lint
Subject to **ament_mypy** (explicit dep in `package.xml`) for the Python helpers, plus the CMake linter for the macros.

## Tests
`colcon test --packages-select nav2_common`

## Docs to update
- `docs.nav2.org` migration guide if `nav2_package()` flags change in user-visible ways (e.g. enabling a warning that breaks downstream third-party plugins).

## Pitfalls
- `nav2_package()` is invoked unconditionally — anything you put in it runs for every package's CMake config phase. Keep it cheap.
- Python helpers are imported by launch files; keep imports lazy and stay compatible with `ament_python` install layouts.
