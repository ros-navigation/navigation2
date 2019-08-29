#!/usr/bin/python3
# Copyright (c) 2019 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# To use this script, run the `test_updown_reliablity` script and log the output
# > ./test_updown_reliablity |& tee /tmp/updown.log
# When the test is completed, pipe the log to this script to get a summary of the
# results
# > ./updownresults.py < /tmp/updown.log
#
# This reports the number of successful tests, but also the number of times the
# tests were able to make it to the active state as well as the shutdown state.
# It can frequently occur that the system makes all the lifecycle state transitions
# but has an error during the final process termination.

import sys

first_error_database = {
    '10: [gzserver-1] [ERROR] [rclcpp]: Couldn\'t add guard_condition to wait set: guard_conditions set is full, at /home/crdelsey/src/ros_dev_workspace/ros2_ws/src/ros2/rcl/rcl/src/rcl/wait.c': 0,
    '10: [test_system_node-12] [ERROR] [nav2_tester]: Robot timed out reaching its goal!': 0,
    '10: [amcl-6] [ERROR] []: Unable to start transition 3 from current state active: Transition is not registered., at /home/crdelsey/src/ros_dev_workspace/ros2_ws/src/ros2/rcl/rcl_lifecycle/src/rcl_lifecycle.c': 0,
    '10: [dwb_controller-7] malloc_consolidate(): invalid chunk size': 0,
    '10: [test_system_node-12] [ERROR] [nav2_tester]: Error couldn\'t set use_sim_time param on:': 0,
    '10: [dwb_controller-7] [ERROR] [rclcpp]: Couldn\'t initialize rcl timer handle: callback/user_data are already added to this clock, at /home/crdelsey/src/ros_dev_workspace/ros2_ws/src/ros2/rcl/rcl/src/rcl/time.c': 0
}

def main():
    log = sys.stdin
    test_count = 0
    fail_count = 0
    successful_bringup_count = 0
    successful_shutdown_count = 0

    test_finished = False
    first_error = False
    for line in log.readlines():
        stripped_line = line.strip()
        if stripped_line.startswith('Start 10: test_bt_navigator'):
            test_successful = False
            shutdown_successful = False
            bringup_successful = False
            first_error = True
            test_finished = False

        if stripped_line.endswith('TOTAL FAILURES'):
            test_count += 1
            conclusion = ''
            if bringup_successful:
                successful_bringup_count += 1
                conclusion = ' but bringup was successful'
            if shutdown_successful:
                successful_shutdown_count += 1
                conclusion = ' but shutdown was successful'
            if not test_successful:
                fail_count += 1
                print('Failure in test ', test_count, conclusion)

        if '[nav2_tester]: Test PASSED' in line:
            test_finished = True

        if stripped_line.startswith('1/1 Test #10: test_bt_navigator ................   Passed'):
            test_successful = True

        if 'The system is active' in line:
            bringup_successful = True

        if 'The system has been sucessfully shut down' in line:
            shutdown_successful = True

        if test_finished or not first_error:
            continue # ignore ERRORs that occur after test has passed and secondary errors
        else:
            key = lookupErrorLine(stripped_line)
            if key:
                first_error = False
                first_error_database[key] = first_error_database[key] + 1
            elif '[ERROR]' in line:
                if 'No valid trajectories' in line:
                    continue # recovery actions should deal with this
                elif '10: [world_model-5] [ERROR] [getCurrentPose]: Extrapolation Error looking up robot pose:' in line:
                    continue
                else:
                    print("Unknown error in test", test_count + 1)
                    print(line)

    print('Number of tests: ', test_count)
    print('Number of successes: ', test_count-fail_count)
    print('Number of successful bringups', successful_bringup_count)
    print('Number of successful shutdowns', successful_shutdown_count)

    for error in first_error_database:
        print(first_error_database[error], '/', test_count, '\t', end='', sep='')
        print(error)

def lookupErrorLine(line):
    for failure in first_error_database:
        if line.startswith(failure):
            return failure

if __name__ == '__main__':
    main()
