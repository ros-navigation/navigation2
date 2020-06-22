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

#include "gtest/gtest.h"
#include "dwb_plugins/one_d_velocity_iterator.hpp"

using dwb_plugins::OneDVelocityIterator;

const double EPSILON = 1e-3;

TEST(VelocityIterator, basics)
{
  OneDVelocityIterator it(2.0, 0.0, 5.0, 1.0, -1.0, 1.0, 2);
  EXPECT_FALSE(it.isFinished());
  EXPECT_NEAR(it.getVelocity(), 1.0, EPSILON);
  EXPECT_FALSE(it.isFinished());
  ++it;
  EXPECT_FALSE(it.isFinished());
  EXPECT_NEAR(it.getVelocity(), 3.0, EPSILON);
  EXPECT_FALSE(it.isFinished());
  ++it;
  EXPECT_TRUE(it.isFinished());
  it.reset();
  EXPECT_FALSE(it.isFinished());
  EXPECT_NEAR(it.getVelocity(), 1.0, EPSILON);
}

TEST(VelocityIterator, limits)
{
  OneDVelocityIterator it(2.0, 1.5, 2.5, 1.0, -1.0, 1.0, 2);
  EXPECT_NEAR(it.getVelocity(), 1.5, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 2.5, EPSILON);
}

TEST(VelocityIterator, acceleration)
{
  OneDVelocityIterator it(2.0, 0.0, 5.0, 0.5, -0.5, 1.0, 2);
  EXPECT_NEAR(it.getVelocity(), 1.5, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 2.5, EPSILON);
}


TEST(VelocityIterator, time)
{
  OneDVelocityIterator it(2.0, 0.0, 5.0, 1.0, -1.0, 0.5, 2);
  EXPECT_NEAR(it.getVelocity(), 1.5, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 2.5, EPSILON);
}

TEST(VelocityIterator, samples)
{
  OneDVelocityIterator it(2.0, 0.0, 5.0, 1.0, -1.0, 1.0, 3);
  EXPECT_NEAR(it.getVelocity(), 1.0, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 2.0, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 3.0, EPSILON);
  ++it;
  EXPECT_TRUE(it.isFinished());
}


TEST(VelocityIterator, samples2)
{
  OneDVelocityIterator it(2.0, 0.0, 5.0, 1.0, -1.0, 1.0, 5);
  EXPECT_NEAR(it.getVelocity(), 1.0, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 1.5, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 2.0, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 2.5, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 3.0, EPSILON);
  ++it;
  EXPECT_TRUE(it.isFinished());
}

TEST(VelocityIterator, around_zero)
{
  OneDVelocityIterator it(0.0, -5.0, 5.0, 1.0, -1.0, 1.0, 2);
  EXPECT_NEAR(it.getVelocity(), -1.0, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 0.0, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 1.0, EPSILON);
  ++it;
}

TEST(VelocityIterator, around_zero2)
{
  OneDVelocityIterator it(0.0, -5.0, 5.0, 1.0, -1.0, 1.0, 4);
  EXPECT_NEAR(it.getVelocity(), -1.0, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), -0.3333, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 0.0, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 0.3333, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 1.0, EPSILON);
  ++it;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
