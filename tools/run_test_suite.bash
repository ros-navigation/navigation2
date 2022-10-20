#!/bin/bash

set -ex

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"  # gets the directory of this script

# Skip flaky tests. Nav2 system tests will be run later.
colcon test --packages-skip nav2_system_tests nav2_behaviors

# run the stable tests in nav2_behaviors
colcon test --packages-select nav2_behaviors --ctest-args --exclude-regex "test_recoveries"

# run the linters in nav2_system_tests. They only need to be run once.
colcon test --packages-select nav2_system_tests --ctest-args --exclude-regex "test_.*"  # run the linters

# Each of the `colcon test` lines above runs tests on independent sets of packages.
# As a result the test logs of each line won't overwrite the others. The single
# call to `colcon test-result` will look through all packages and report any errors
# that happened in any of the `colcon test` lines above.
colcon test-result --verbose

# $SCRIPT_DIR/ctest_retry.bash -r 3 -d build/nav2_system_tests -t test_localization$
# $SCRIPT_DIR/ctest_retry.bash -r 3 -d build/nav2_system_tests -t test_planner_costmaps$
# $SCRIPT_DIR/ctest_retry.bash -r 3 -d build/nav2_system_tests -t test_planner_random$
# $SCRIPT_DIR/ctest_retry.bash -r 3 -d build/nav2_system_tests -t test_bt_navigator$
# $SCRIPT_DIR/ctest_retry.bash -r 3 -d build/nav2_system_tests -t test_bt_navigator_with_dijkstra$
$SCRIPT_DIR/ctest_retry.bash -r 3 -d build/nav2_system_tests -t test_dynamic_obstacle$
# $SCRIPT_DIR/ctest_retry.bash -r 3 -d build/nav2_system_tests -t test_multi_robot$
