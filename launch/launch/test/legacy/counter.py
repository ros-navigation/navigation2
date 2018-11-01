#!/usr/bin/env python3

# Copyright 2015 Open Source Robotics Foundation, Inc.
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

import argparse
import signal
import sys
import time


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description=(
            'Simple program outputting a counter. '
            'Even values go to STDERR, odd values go to STDOUT.'))
    parser.add_argument(
        '--limit',
        type=int,
        help='The upper limit of the counter when the program terminates.')
    parser.add_argument(
        '--limit-return-code',
        type=int,
        default=0,
        help='The return code when the program terminates because of reaching '
             'the limit.')
    parser.add_argument(
        '--sleep',
        type=float,
        default=1.0,
        help='The time to sleep between a counter increment.')

    args = parser.parse_args(argv)

    if args.limit:
        signal.signal(signal.SIGINT, lambda signum, frame: print('ignoring SIGINT'))

    counter = 1
    while True:
        if args.limit is not None and counter > args.limit:
            return args.limit_return_code
        stream = sys.stdout if counter % 2 else sys.stderr
        # print('Counter: %d' % counter, file=stream)
        stream.write('Counter: ')
        stream.write('%d\n' % counter)
        time.sleep(args.sleep)
        counter += 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
