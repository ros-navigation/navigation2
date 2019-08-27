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


def main():
    log = sys.stdin
    test_count = 0
    fail_count = 0
    successful_bringup_count = 0
    successful_shutdown_count = 0
    for line in log.readlines():
        if line.startswith('======= START OF RUN:'):
            test_successful = True
            shutdown_successful = False
            bringup_successful = False

        if line.startswith('======== END OF RUN:'):
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

        if '[ERROR]' in line:
            test_successful = False

        if 'The system is active' in line:
            bringup_successful = True

        if 'The system has been sucessfully shut down' in line:
            shutdown_successful = True

    print('Number of tests: ', test_count)
    print('Number of successes: ', test_count-fail_count)
    print('Number of successful bringups', successful_bringup_count)
    print('Number of successful shutdowns', successful_shutdown_count)


if __name__ == '__main__':
    main()
