# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Module for the create_future() utility function."""

import asyncio


def create_future(loop: asyncio.AbstractEventLoop) -> asyncio.Future:
    """
    Return a Future, using the loop if possible.

    loop.create_future() is better, but was only added in Python 3.5.2, see:

    https://docs.python.org/3/library/asyncio-eventloop.html#asyncio.AbstractEventLoop.create_future
    """
    if hasattr(loop, 'create_future'):
        return loop.create_future()
    return asyncio.Future(loop=loop)
