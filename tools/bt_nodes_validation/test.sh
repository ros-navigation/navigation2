#!/bin/bash
# Wrapper script to run tests with clean environment.

# Unset PYTHONPATH to isolate tests from ROS2 packages.
# The setup.bash adds site-packages to PYTHONPATH,
# which includes ROS2 dependencies not needed for tests, leading to import errors.

unset PYTHONPATH

cd "$(dirname "$0")"
pytest .
