#!/bin/bash

set -ex

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"  # gets the directory of this script

colcon test --packages-skip nav2_system_tests
colcon test --packages-select nav2_system_tests --ctest-args --exclude-regex "test_.*"  # run the linters
colcon test-result --verbose
$SCRIPT_DIR/ctest_retry.bash -r 3 -d build/nav2_system_tests -t test_localization
$SCRIPT_DIR/ctest_retry.bash -r 3 -d build/nav2_system_tests -t test_simple_navigator
$SCRIPT_DIR/ctest_retry.bash -r 3 -d build/nav2_system_tests -t test_bt_navigator
