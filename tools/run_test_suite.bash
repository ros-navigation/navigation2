#!/bin/bash

set -ex

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"  # gets the directory of this script

# Skip flaky tests. Nav2 system tests will be run later.
colcon test --packages-skip nav2_system_tests nav2_dynamic_params nav2_motion_primitives \
                            nav2_bt_navigator nav2_costmap_2d nav2_lifecycle_manager \
                            nav2_tasks nav2_robot
                            
# run the stable tests in nav2_dynamic_params
colcon test --packages-select nav2_dynamic_params --ctest-args --exclude-regex "test_dynamic_params_client"

# run the stable tests in nav2_motion_primitives
colcon test --packages-select nav2_motion_primitives --ctest-args --exclude-regex "test_motion_primitives"

# run the linters in nav2_system_tests. They only need to be run once.
colcon test --packages-select nav2_system_tests --ctest-args --exclude-regex "test_.*"  # run the linters

# Each of the `colcon test` lines above runs tests on independent sets of packages.
# As a result the test logs of each line won't overwrite the others. The single
# call to `colcon test-result` will look through all packages and report any errors
# that happened in any of the `colcon test` lines above.
colcon test-result --verbose

$SCRIPT_DIR/ctest_retry.bash -r 3 -d build/nav2_system_tests -t test_localization
$SCRIPT_DIR/ctest_retry.bash -r 3 -d build/nav2_system_tests -t test_bt_navigator
