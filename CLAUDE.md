# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Navigation2 Architecture

Navigation2 is a comprehensive ROS2 navigation framework organized as a metapackage containing multiple specialized packages:

### Core Navigation Stack
- **nav2_planner**: Global path planning server with pluggable algorithms (NavFn, Smac, Theta*)
- **nav2_controller**: Local control server with pluggable controllers (DWB, RPP, MPPI, Graceful)
- **nav2_bt_navigator**: Behavior tree-based navigation coordinator
- **nav2_lifecycle_manager**: Manages lifecycle transitions across navigation nodes
- **nav2_costmap_2d**: 2D costmap representation with layered architecture
- **nav2_smoother**: Path smoothing algorithms

### Specialized Components
- **nav2_amcl**: Adaptive Monte Carlo Localization
- **nav2_map_server**: Map loading/saving and serving
- **nav2_behavior_tree**: Behavior tree framework integration
- **nav2_behaviors**: Recovery behaviors (spin, backup, wait)
- **nav2_collision_monitor**: Real-time collision detection and avoidance
- **nav2_waypoint_follower**: Sequential waypoint navigation
- **nav2_velocity_smoother**: Velocity command smoothing
- **nav2_route**: Route based planning and tracking
- **nav2_voxel_grid**: 3D voxel grid implementation for costmap layers
- **nav2_loopback_sim**: Python-based loopback physics simulator for testing

### Foundation and Support
- **navigation2**: Metapackage that aggregates all Navigation2 packages
- **nav2_core**: Core plugin interfaces and base classes for planners, controllers, and behaviors
- **nav2_common**: Common support functionality and CMake utilities
- **nav2_ros_common**: Common ROS utilities and helper functions
- **nav2_msgs**: Message and service definitions for Navigation2
- **nav2_rviz_plugins**: RViz2 visualization plugins for navigation display

### Emerging Components
- **opennav_docking**: Robot charger docking task server with pluggable dock implementations
- **opennav_docking_core**: Core plugin interfaces for docking algorithms
- **opennav_docking_bt**: Behavior tree nodes for docking operations
- **opennav_following**: Dynamic object following task server for tracking moving targets

## Plugin Algorithms

### Controller Plugins
- **nav2_dwb_controller**: DWB local planner with customizable cost functions
  - **dwb_core**: Core DWB interfaces and trajectory generation
  - **dwb_msgs**: DWB-specific message definitions
  - **dwb_plugins**: Standard DWB plugin implementations (trajectory generators, goal checkers)
  - **dwb_critics**: Trajectory evaluation critics (oscillation, obstacles, path alignment)
  - **costmap_queue**: Efficient costmap traversal utilities for DWB
  - **nav_2d_msgs**: 2D navigation-specific message types
  - **nav_2d_utils**: 2D navigation utility functions and conversions
- **nav2_mppi_controller**: MPPI local planner with model predictive control
- **nav2_graceful_controller**: Graceful local planner for smooth navigation with spiral curves
- **nav2_regulated_pure_pursuit_controller**: Regulated Pure Pursuit local planner
- **nav2_rotation_shim_controller**: Rotation shim controller for smooth turns

### Planner Plugins
- **nav2_smac_planner**: SMAC global planner with adaptive heuristics
- **nav2_theta_star_planner**: Theta* global planner with adaptive heuristics
- **nav2_navfn_planner**: NavFn global planner with A* search

### Smoother Plugins
- **nav2_constrained_smoother**: Ceres Optimization-based Constrained path smoothing

### Development and Testing
- **nav2_system_tests**: Integration tests and system-level validation
- **nav2_bringup**: Launch files and configuration examples
- **nav2_simple_commander**: Python API for navigation control
- **nav2_util**: Common utilities and helper functions

## Build System

Navigation2 uses the ROS2 colcon build system. From the workspace root (`/home/steve/Documents/rolling_ws`):

### Build Commands
```bash
# Build all packages
colcon build --parallel-workers 2

# Build specific package
colcon build --packages-select nav2_planner

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Build with tests
colcon build --cmake-args -DBUILD_TESTING=ON
```

### Testing Commands
```bash
# Run all tests (excluding flaky system tests)
colcon test

# Run tests for specific package
colcon test --packages-select nav2_planner

# Run linters only
colcon test --packages-select nav2_system_tests --ctest-args --exclude-regex "test_.*"

# View test results
colcon test-result --verbose
```

## Code Quality and Linting

### Pre-commit Hooks
The project uses pre-commit for automated code quality checks:
```bash
# Install pre-commit hooks
pre-commit install

# Run all checks manually
pre-commit run -a
```

### Linting Tools
- **ament_cpplint**: C++ code style checking
- **ament_uncrustify**: C++ code formatting (auto-reformats with `--reformat`)
- **ament_flake8**: Python style checking  
- **ament_pep257**: Python docstring checking
- **ament_mypy**: Python type checking (config in `tools/pyproject.toml`)
- **ament_xmllint**: XML markup validation

### Manual Linting
```bash
# Run specific linter
ament_cpplint <file>
ament_uncrustify --reformat <file>
ament_flake8 <file>
ament_mypy --config tools/pyproject.toml <file>
```

## Plugin Architecture

Navigation2 uses ROS2 pluginlib for extensible algorithms. Plugin declarations are in XML files:
- Controllers: `*_controller.xml` 
- Planners: `*_planner.xml`
- Behavior tree nodes: `nav2_tree_nodes.xml`
- Costmap layers: `costmap_plugins.xml`

## Development Guidelines

### Package Structure
Each nav2 package follows standard ROS2 conventions:
- `package.xml`: Package metadata and dependencies
- `CMakeLists.txt`: Build configuration
- `include/`: Header files
- `src/`: Source files
- `plugins/`: Plugin implementations
- `test/`: Unit and integration tests

### Code Conventions
- C++ code follows ROS2 style guidelines enforced by uncrustify
- Python code follows Google style (configured in `tools/pyproject.toml`)
- All commits must be signed-off with DCO (Developer Certificate of Origin)
- Behavior trees are defined in XML format under `behavior_trees/`
- Make sure to NEVER add white space at the end of lines and ALWAYS add a newline at the end of files.
- Additions should have unit or system tests
- Lines MUST be less than 100 characters long

### Key Configuration Files
- `.pre-commit-config.yaml`: Pre-commit hook configuration
- `tools/pyproject.toml`: Python tooling configuration (mypy, isort, codespell)
- `codecov.yml`: Code coverage reporting configuration

### Behavior Tree Development
- Behavior tree nodes are implemented as plugins
- XML tree definitions in `nav2_bt_navigator/behavior_trees/`
- Custom nodes registered in `nav2_tree_nodes.xml`
- Use `tools/bt2img.py` to generate tree diagrams

## Development Tools

The `tools/` directory contains essential scripts and utilities for development:

### Testing and Quality Assurance
- **ctest_retry.bash**: Retry utility for flaky system tests
- **run_test_suite.bash**: Automated test suite runner
- **code_coverage_report.bash**: Generate and format code coverage reports
- **run_sanitizers/**: Memory and address sanitizer test runners

### Documentation and Visualization
- **bt2img.py**: Convert behavior tree XML to visual diagrams
- **update_bt_diagrams.bash**: Bulk update all BT diagrams in documentation
- **update_readme_table.py**: Automatically update package tables in README

### Benchmarking
- **planner_benchmarking/**: Performance benchmarking tools for path planners
- **smoother_benchmarking/**: Performance benchmarking tools for path smoothers

### Configuration
- **pyproject.toml**: Python tooling configuration (mypy, isort, codespell, flake8)
- **.codespell_ignore_words**: Custom dictionary for codespell linter
- **underlay.repos**: VCS repository configuration for dependencies

### Docker
- **distro.Dockerfile**: Build Navigation2 from distribution packages
- **source.Dockerfile**: Build Navigation2 from source

## Testing Strategy

### Test Types
- Unit tests for individual classes/functions
- Integration tests for component interactions  
- System tests for end-to-end navigation scenarios (often flaky, run separately)
- Linting tests for code quality

### Running Specific Tests
- Use `tools/ctest_retry.bash` for flaky test retries

## Important Development Notes

- The workspace contains a full colcon workspace structure with `build/`, `install/`, and `log/` directories
- Documentation is maintained separately at docs.nav2.org
- This is a rolling development branch (ROS2 Rolling distribution)
- Code coverage reports are generated and uploaded to codecov.io
- Professional support available through Open Navigation LLC