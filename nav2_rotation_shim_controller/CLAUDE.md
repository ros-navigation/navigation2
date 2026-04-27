# nav2_rotation_shim_controller — assistant guide

Implements `nav2_core::Controller` as a **shim** that rotates the robot toward a path heading before delegating to a primary controller. Not a standalone controller — always paired with another.

## Interface
`nav2_core::Controller` — hosted by `nav2_controller::ControllerServer`.

## Plugin XML
- `nav2_rotation_shim_controller.xml` → `nav2_rotation_shim_controller::RotationShimController`.

## Layout
- `include/nav2_rotation_shim_controller/` — `nav2_rotation_shim_controller.hpp`
- `src/` — shim logic + delegation
- `test/`

## Wiring
The primary controller is declared as a sub-plugin under the shim's params (`primary_controller`). Both must be loadable by `ControllerServer`. See `nav2_bringup/params/nav2_params.yaml` for the canonical param block.

## How to change behavior
- Heading-error tolerance, angular velocity → params in `nav2_bringup/params/nav2_params.yaml`.
- Hand-off condition → `src/nav2_rotation_shim_controller.cpp` (logic for switching from shim's rotate to delegated control).

## Tests
`colcon test --packages-select nav2_rotation_shim_controller`

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-rotation-shim-controller.md`.

## Pitfalls
- The shim **owns** the primary controller's lifetime — if you allocate state in the shim's `configure`, also propagate `activate`/`deactivate` to the primary.
- Misconfigured `angular_dist_threshold` causes oscillation between shim and primary — surface clearly when it changes.
