/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/




#include <gtest/gtest.h>

#include <base_local_planner/velocity_iterator.h>


namespace base_local_planner {

TEST(VelocityIteratorTest, testsingle) {
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIterator x_it(0.0, 0.0, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(1, i);
  EXPECT_EQ(0, result[0]);
}

TEST(VelocityIteratorTest, testsingle_pos) {
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIterator x_it(2.2, 2.2, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(1, i);
  EXPECT_EQ(2.2, result[0]);
}

TEST(VelocityIteratorTest, testsingle_neg) {
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIterator x_it(-3.3, -3.3, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(1, i);
  EXPECT_EQ(-3.3, result[0]);
}

TEST(VelocityIteratorTest, test1) {
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIterator x_it(-30, 30, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(3, i);
  double expected [3]= {-30.0, 0.0, 30.0};
  for (int j = 0; j < 3; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

TEST(VelocityIteratorTest, test1_pos) {
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIterator x_it(10, 30, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(2, i);
  double expected [2]= {10.0, 30.0};
  for (int j = 0; j < 2; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

TEST(VelocityIteratorTest, test1_neg) {
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIterator x_it(-30, -10, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(2, i);
  double expected [2]= {-30.0, -10.0};
  for (int j = 0; j < 2; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

TEST(VelocityIteratorTest, test3) {
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIterator x_it(-30, 30, 3); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(3, i);
  double expected [3]= {-30.0, 0.0, 30};
  for (int j = 0; j < 3; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

TEST(VelocityIteratorTest, test4) {
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIterator x_it(-30, 30, 4); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(5, i);
  double expected [5]= {-30.0, -10.0, 0.0, 10.0, 30};
  for (int j = 0; j < 5; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

TEST(VelocityIteratorTest, test_shifted) {
  // test where zero is not in the middle
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIterator x_it(-10, 50, 4); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(5, i);
  double expected [5]= {-10.0, 0.0, 10.0, 30, 50};
  for (int j = 0; j < 5; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

TEST(VelocityIteratorTest, test_cranky) {
  // test where one value is almost zero (nothing to do about that)
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIterator x_it(-10.00001, 10, 3); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(4, i);
  for (int j = 0; j < 5; ++j) {
	double expected [5]= {-10.00001, -0.000005, 0.0, 10.0};
    EXPECT_FLOAT_EQ(expected[j], result[j]);
  }
}

} // namespace
