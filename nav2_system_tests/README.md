# System Tests

The package provides tests for [components](#1.-Component-Testing), [subsystems](#2.-Subsystem-Testing) and full [system](#3.-System-Testing) integration.

Unit tests are not included, these should be provided within each component or package.

## 1. Component Testing
Test a component's ROS API (Pub/Sub/Service).

- [Global Planning](src/planning/README.md)

- Controller

- [Localization](src/localization/README.md)

- World Model

- Costmaps

## 2. Subsystem Testing
Test the integration of several components and subsystems.

- Support modules (Mapping, Perception, Prediction, Localization

- Navigation core (Navigator, Planner, Controller)

- Support modules and navigation core

- Command chain (Mission Planning, Mission Execution, Navigation System, Robot Interface)

## 3. System Testing
Test the lifecycle startup and shutdown of nodes.
 - [Updown Test](src/updown/README.md)

Test the integration of all subsystems.
 - [System Test](src/system/README.md)
