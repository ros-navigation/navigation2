#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright 2015 Martin Llofriu, Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rclpy
import unittest
import time
from builtin_interfaces.msg import Time
from std_msgs.msg import String

from message_filters import Cache, Subscriber, stamp_time
from rclpy.constants import S_TO_NS


PKG = 'message_filters'


class AnonymMsg:
    class AnonymHeader:
        stamp = None

        def __init__(self):
            stamp = time.monotonic()

    header = None

    def __init__(self):
        self.header = AnonymMsg.AnonymHeader()


class TestCache(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('my_node', namespace='/my_ns')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_all_funcs(self):
        sub = Subscriber(self.node, String, "/empty")
        cache = Cache(sub, 5)

        msg = AnonymMsg()
        msg.header.stamp = Time(sec=0)
        cache.add(msg)

        msg = AnonymMsg()
        msg.header.stamp = Time(sec=1)
        cache.add(msg)

        msg = AnonymMsg()
        msg.header.stamp = Time(sec=2)
        cache.add(msg)

        msg = AnonymMsg()
        msg.header.stamp = Time(sec=3)
        cache.add(msg)

        msg = AnonymMsg()
        msg.header.stamp = Time(sec=4)
        cache.add(msg)

        l = len(cache.getInterval(2.5, 3.5))
        self.assertEqual(l, 1, "invalid number of messages" +
                                " returned in getInterval 1")

        l = len(cache.getInterval(2, 3))
        self.assertEqual(l, 2, "invalid number of messages" +
                                " returned in getInterval 2")

        l = len(cache.getInterval(0, 4))
        self.assertEqual(l, 5, "invalid number of messages" +
                                " returned in getInterval 3")

        s = cache.getElemAfterTime(0).header.stamp
        self.assertEqual(stamp_time(s), 0,
                         "invalid msg return by getElemAfterTime")

        s = cache.getElemBeforeTime(3.5).header.stamp
        self.assertEqual(stamp_time(s), 3,
                         "invalid msg return by getElemBeforeTime")

        s = cache.getLastestTime()
        self.assertEqual(s, 4,
                         "invalid stamp return by getLastestTime")

        s = cache.getOldestTime()
        self.assertEqual(s, 0,
                         "invalid stamp return by getOldestTime")

        # Add another msg to fill the buffer
        msg = AnonymMsg()
        msg.header.stamp = Time(sec=5)
        cache.add(msg)

        # Test that it discarded the right one
        s = cache.getOldestTime()
        self.assertEqual(s, 1,
                         "wrong message discarded")

'''
    def test_headerless(self):
        sub = Subscriber("/empty", String)
        cache = Cache(sub, 5, allow_headerless=False)

        msg = String()
        cache.add(msg)

        self.assertIsNone(cache.getElemAfterTime(Time(sec=0)),
                          "Headerless message invalidly added.")

        cache = Cache(sub, 5, allow_headerless=True)

        rospy.rostime.set_rostime_initialized(True)

        rospy.rostime._set_rostime(Time(sec=0))
        cache.add(msg)

        s = cache.getElemAfterTime(Time(sec=0))
        self.assertEqual(s, msg,
                         "invalid msg returned in headerless scenario")

        s = cache.getElemAfterTime(Time(sec=1))
        self.assertIsNone(s, "invalid msg returned in headerless scenario")

        rospy.rostime._set_rostime(Time(sec=2))
        cache.add(msg)

        s = cache.getInterval(Time(sec=0), Time(sec=1))
        self.assertEqual(s, [msg],
                         "invalid msg returned in headerless scenario")

        s = cache.getInterval(Time(sec=0), Time(sec=2))
        self.assertEqual(s, [msg, msg],
                         "invalid msg returned in headerless scenario")
'''

if __name__ == '__main__':
    unittest.main()
