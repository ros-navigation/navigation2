# <PKG_NAME> — assistant guide

<one-line role: "Implements <Interface> for <server_pkg>; ships N plugin variants.">

## Interface
`nav2_core::<Interface>` — `nav2_core/include/nav2_core/<interface>.hpp`. Lifecycle methods (`configure`/`activate`/`cleanup`) are called by the host server; do not allocate ROS entities outside them.

## Plugin XML(s)
- `<plugin_xml_filename>.xml` at package root — one `<class>` block per shipped plugin.
- Exported via `pluginlib_export_plugin_description_file(nav2_core <plugin_xml_filename>.xml)` in `CMakeLists.txt` and `<export><nav2_core plugin="${prefix}/<plugin_xml_filename>.xml"/></export>` in `package.xml`.

## Layout
- `include/<pkg>/` — public headers
- `src/` — implementation
- `test/` — gtest binaries (see below)
- `CMakeLists.txt` calls `nav2_package()` from `nav2_common`

## How to add a new <role> variant
1. Copy the closest existing class as a starting point.
2. Implement the interface in `include/<pkg>/<class>.hpp` + `src/<class>.cpp`.
3. `PLUGINLIB_EXPORT_CLASS(<pkg>::<Class>, nav2_core::<Interface>)` at the bottom of the `.cpp`.
4. Add a `<class>` block to `<plugin_xml_filename>.xml` (matching `type=` and `base_class_type=`).
5. Add the new library target + `install()` to `CMakeLists.txt`.
6. Register the plugin name + parameters in `nav2_bringup/params/nav2_params.yaml` under the host server.
7. Add unit tests under `test/`; mirror the gtest pattern of an existing test.

## Tests
- Canonical binary(ies): `./build/<pkg>/<test_name>`
- `colcon test --packages-select <pkg>`

## Docs to update when touching this package
- `docs.nav2.org` → `configuration/packages/configuring-<pkg>.md` for any param change.
- Plugins page if a new plugin is added.
- Tuning guide if a knob's recommended value or behavior changes.
- Migration guide if behavior is not backwards-compatible.

## Pitfalls
<role-specific gotchas — fill in per package>
