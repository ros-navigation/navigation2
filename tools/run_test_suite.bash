#!/bin/bash

set -ex

colcon test --packages-skip nav2_system_tests
colcon test-result --verbose
cp src/navigation2/tools/ctest_retry.bash build/nav2_system_tests
cd build/nav2_system_tests
./ctest_retry.bash 3
