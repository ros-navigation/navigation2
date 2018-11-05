/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
#include "gtest/gtest.h"
#include "costmap_queue/map_based_queue.hpp"

using costmap_queue::MapBasedQueue;

void letter_test(MapBasedQueue<char> & q, const char test_letter)
{
  ASSERT_FALSE(q.isEmpty());
  char c = q.front();
  EXPECT_EQ(c, test_letter);
  q.pop();
}

TEST(MapBasedQueue, emptyQueue)
{
  MapBasedQueue<char> q;
  EXPECT_TRUE(q.isEmpty());
  q.enqueue(1.0, 'A');
  EXPECT_FALSE(q.isEmpty());
}

TEST(MapBasedQueue, checkOrdering)
{
  MapBasedQueue<char> q;
  q.enqueue(1.0, 'A');
  q.enqueue(3.0, 'B');
  q.enqueue(2.0, 'C');
  q.enqueue(5.0, 'D');
  q.enqueue(0.0, 'E');

  std::string expected = "EACBD";
  for (unsigned int i = 0; i < expected.size(); i++) {
    letter_test(q, expected[i]);
  }
  EXPECT_TRUE(q.isEmpty());
}

TEST(MapBasedQueue, checkDynamicOrdering)
{
  MapBasedQueue<char> q;
  q.enqueue(1.0, 'A');
  q.enqueue(3.0, 'B');
  q.enqueue(2.0, 'C');
  q.enqueue(5.0, 'D');

  std::string expected = "ACB";
  for (unsigned int i = 0; i < expected.size(); i++) {
    letter_test(q, expected[i]);
  }

  q.enqueue(0.0, 'E');
  letter_test(q, 'E');
}

TEST(MapBasedQueue, checkDynamicOrdering2)
{
  MapBasedQueue<char> q;
  q.enqueue(1.0, 'A');
  q.enqueue(2.0, 'B');
  letter_test(q, 'A');
  q.enqueue(3.0, 'C');
  letter_test(q, 'B');
}

TEST(MapBasedQueue, checkDynamicOrdering3)
{
  MapBasedQueue<char> q;
  q.enqueue(1.0, 'A');
  q.enqueue(2.0, 'B');
  q.enqueue(5.0, 'D');
  letter_test(q, 'A');
  letter_test(q, 'B');
  q.enqueue(1.0, 'C');
  letter_test(q, 'C');
  letter_test(q, 'D');
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
