#!/bin/bash

set -ex

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"  # gets the directory of this script

# Skip nav2_system_tests because the system tests are flaky and need to be retried
# a few times.
#
# Skip the nav2_dynamic_params tests because they fail when run concurrently with
# other tests. We could run all the tests sequentially to fix this, however, by
# just deferring this one package and running it later, we don't have to slow everything down.
colcon test --packages-skip nav2_system_tests nav2_dynamic_params

# run the stable tests in nav2_dynamic_params
colcon test --packages-select nav2_dynamic_params --ctest-args --exclude-regex "test_dynamic_params_client"

# run the linters in nav2_system_tests. They only need to be run once.
colcon test --packages-select nav2_system_tests --ctest-args --exclude-regex "test_.*"  # run the linters

# Each of the `colcon test` lines above runs tests on independent sets of packages.
# As a result the test logs of each line won't overwrite the others. The single
# call to `colcon test-result` will look through all packages and report any errors
# that happened in any of the `colcon test` lines above.
colcon test-result --verbose

$SCRIPT_DIR/ctest_retry.bash -r 3 -d build/nav2_dynamic_params -t test_dynamic_params_client
$SCRIPT_DIR/ctest_retry.bash -r 3 -d build/nav2_system_tests -t test_localization
$SCRIPT_DIR/ctest_retry.bash -r 3 -d build/nav2_system_tests -t test_simple_navigator
$SCRIPT_DIR/ctest_retry.bash -r 3 -d build/nav2_system_tests -t test_bt_navigator
