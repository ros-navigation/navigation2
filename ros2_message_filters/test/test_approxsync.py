#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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
import random
import time
from builtin_interfaces.msg import Time

import message_filters
from message_filters import ApproximateTimeSynchronizer

class MockHeader:
    pass

class MockMessage:
    def __init__(self, stamp, data):
        self.header = MockHeader()
        self.header.stamp = stamp
        self.data = data

class MockHeaderlessMessage:
    def __init__(self, data):
        self.data = data

class MockFilter(message_filters.SimpleFilter):
    pass

class TestApproxSync(unittest.TestCase):

    def cb_collector_2msg(self, msg1, msg2):
        self.collector.append((msg1, msg2))

    def test_approx(self):
        m0 = MockFilter()
        m1 = MockFilter()
        ts = ApproximateTimeSynchronizer([m0, m1], 1, 0.1)
        ts.registerCallback(self.cb_collector_2msg)

        if 0:
            # Simple case, pairs of messages, make sure that they get combined
            for t in range(10):
                self.collector = []
                msg0 = MockMessage(t, 33)
                msg1 = MockMessage(t, 34)
                m0.signalMessage(msg0)
                self.assertEqual(self.collector, [])
                m1.signalMessage(msg1)
                self.assertEqual(self.collector, [(msg0, msg1)])

        # Scramble sequences of length N.  Make sure that TimeSequencer recombines them.
        random.seed(0)
        for N in range(1, 10):
            m0 = MockFilter()
            m1 = MockFilter()
            seq0 = [MockMessage(Time(sec=t), random.random()) for t in range(N)]
            seq1 = [MockMessage(Time(sec=t), random.random()) for t in range(N)]
            # random.shuffle(seq0)
            ts = ApproximateTimeSynchronizer([m0, m1], N, 0.1)
            ts.registerCallback(self.cb_collector_2msg)
            self.collector = []
            for msg in random.sample(seq0, N):
                m0.signalMessage(msg)
            self.assertEqual(self.collector, [])
            for msg in random.sample(seq1, N):
                m1.signalMessage(msg)
            self.assertEqual(set(self.collector), set(zip(seq0, seq1)))

        # Scramble sequences of length N of headerless and header-having messages.
        # Make sure that TimeSequencer recombines them.
#        rospy.rostime.set_rostime_initialized(True)
'''
        random.seed(0)
        for N in range(1, 10):
            m0 = MockFilter()
            m1 = MockFilter()
            seq0 = [MockMessage(Time(sec=t), random.random()) for t in range(N)]
            seq1 = [MockHeaderlessMessage(random.random()) for t in range(N)]
            # random.shuffle(seq0)
            ts = ApproximateTimeSynchronizer([m0, m1], N, 0.1, allow_headerless=True)
            ts.registerCallback(self.cb_collector_2msg)
            self.collector = []
            for msg in random.sample(seq0, N):
                m0.signalMessage(msg)
            self.assertEqual(self.collector, [])
            for i in random.sample(range(N), N):
                msg = seq1[i]
               rospy.rostime._set_rostime(rospy.Time(i+0.05))
                time.sleep(0.05)
                m1.signalMessage(msg)
            self.assertEqual(set(self.collector), set(zip(seq0, seq1)))
'''

if __name__ == '__main__':
    suite = unittest.TestSuite()
    suite.addTest(TestApproxSync('test_approx'))
    unittest.TextTestRunner(verbosity=2).run(suite)
