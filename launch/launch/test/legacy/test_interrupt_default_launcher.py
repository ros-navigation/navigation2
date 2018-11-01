# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import _thread
import sys
import threading

from launch.legacy import LaunchDescriptor
from launch.legacy.launcher import DefaultLauncher


def test_interrupt_default_launcher():
    desc = LaunchDescriptor()
    desc.add_process(
        cmd=[sys.executable, '-c', 'import time', 'time.sleep(30)'],
        name='test_interrupt_default_launcher__python_blocking'
    )

    launcher = DefaultLauncher()
    launcher.add_launch_descriptor(desc)

    def thread_target():
        # wait up to 10 seconds to get to the point where at least all of the
        # asyncio-subprocess coroutines have been run (the processes are still
        # not guaranteed to be running yet)
        launcher.wait_on_processes_to_spawn(10)
        if not launcher.are_processes_spawned():
            # if the processes didn't launch after 10 seconds, fail the test
            _thread.interrupt_main()
            assert False, 'launcher never reported processes launched'
        launcher.interrupt_launch()
        # now wait for the launcher to finish and error if if doesn't
        launcher.wait_on_launch_to_finish(60)
        if launcher.is_launch_running():
            # if still running fail the test
            _thread.interrupt_main()
            assert False, 'launcher failed to shutdown'

    t = threading.Thread(target=thread_target)
    t.start()

    try:
        launcher.launch()
    except KeyboardInterrupt:
        assert False, 'failed by watchdog thread, see other AssertionErrors'
    t.join()
